# SPDX-FileCopyrightText: Copyright (c) 2024 Tod Kurt
# SPDX-FileCopyrightText: Copyright (c) 2024 Tom Whitwell
#
# SPDX-License-Identifier: MIT
"""
`mtm_computer`
================================================================================

Library for Music Thing Modular Computer

* Author(s): Tod Kurt, Vincent Maurer, based off Arduino code from Tom Whitwell and Chris Jones ComputerCard library

"""

import board
import digitalio
import analogio
import pwmio
import busio
import audiopwmio 
import audiomixer
import math
import time
# import struct

# Rough calibrations from one device
#CVzeroPoint = 2085  # 0v
#CVlowPoint = 100;  # -6v 
#CVhighPoint = 4065  # +6v

CVzeroPoint = 2085 * 16  # 0v  (circuitpython duty-cycle is 16-bit always)
CVlowPoint = 100 * 16;  # -6v 
CVhighPoint = 4065 * 16  # +6v

# NB MCP4822 wrongly configured - -5 to +5v, was supposed to be +-6
DACzeroPoint = 1657  # 0v
DAClowPoint = 3031  # -5v 
DAChighPoint = 281  # +5v


# ADC input pins
AUDIO_1_IN_PIN = board.GP27
AUDIO_2_IN_PIN = board.GP26
MUX_IO_1 = board.A2
MUX_IO_2 = board.A3

# Mux pins
MUX_LOGIC_A = board.GP24
MUX_LOGIC_B = board.GP25

# Pulse pins
PULSE_1_INPUT = board.GP2
PULSE_2_INPUT = board.GP3

# Output pins
PULSE_1_RAW_OUT = board.GP8
PULSE_2_RAW_OUT = board.GP9
CV_1_PWM = board.GP23
CV_2_PWM = board.GP22

# DAC pins
DAC_CS = board.GP21
DAC_SDI = board.GP19
DAC_SCK = board.GP18

# DAC parameters (Buffered, 1x Gain: 0011 / 1011)
DAC_config_chan_A_gain = 0b0011000000000000
DAC_config_chan_B_gain = 0b1011000000000000

LED_PINS = (board.GP10, board.GP11, board.GP12, board.GP13, board.GP14, board.GP15)

CHANNEL_COUNT = 8  # three knobs, 1 switch, 2 CV (= six? I guess two analog ins?)


def map_range(s, a1, a2, b1, b2):
    """Like Arduino ``map()``"""
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

def gamma_correct(x):
    """simple and dumb LED brightness gamma-correction"""
    return min(max(int((x*x)/65535), 0), 65535)

class Computer:
    """
    Computer creates appropriate CircuitPython objects for all I/O devices.
    """
    def __init__(self):
        self.analog = [0] * CHANNEL_COUNT
        self.analog_mux1 = analogio.AnalogIn(MUX_IO_1)
        self.analog_mux2 = analogio.AnalogIn(MUX_IO_2)
        # audio inputs
        self.audio_1_in_read = analogio.AnalogIn(AUDIO_1_IN_PIN)
        self.audio_2_in_read = analogio.AnalogIn(AUDIO_2_IN_PIN)
        
        self.mux_count = 0
        self.analog_smooth_amount = 0.3
        
        # Internal state for dac
        self._audio_1_out_value = 0
        self._audio_2_out_value = 0

        self.leds = []
        for pin in LED_PINS:
            d = pwmio.PWMOut(pin, frequency=60_000, duty_cycle=0)
            self.leds.append(d)

        self.mux_A = digitalio.DigitalInOut(MUX_LOGIC_A)
        self.mux_B = digitalio.DigitalInOut(MUX_LOGIC_B)
        self.mux_A.switch_to_output()
        self.mux_B.switch_to_output()

        self.pulse_1_in_read = digitalio.DigitalInOut(PULSE_1_INPUT)
        self.pulse_2_in_read = digitalio.DigitalInOut(PULSE_2_INPUT)
        self.pulse_1_in_read.switch_to_input(pull=digitalio.Pull.UP)
        self.pulse_2_in_read.switch_to_input(pull=digitalio.Pull.UP)

        # Store pulse out DigitalInOut objects as private attributes to allow pulse_outs_to_audio to deinit them
        self._pulse_1_out_pin = digitalio.DigitalInOut(PULSE_1_RAW_OUT)
        self._pulse_2_out_pin = digitalio.DigitalInOut(PULSE_2_RAW_OUT)
        self._pulse_1_out_pin.switch_to_output(value=True) 
        self._pulse_2_out_pin.switch_to_output(value=True)

        self.cv_1_pwm = pwmio.PWMOut(CV_1_PWM, frequency=60_000, duty_cycle=CVzeroPoint)
        self.cv_2_pwm = pwmio.PWMOut(CV_2_PWM, frequency=60_000, duty_cycle=CVzeroPoint)

        self.dac_spi = busio.SPI(clock=DAC_SCK, MOSI=DAC_SDI)
        if self.dac_spi.try_lock():
            self.dac_spi.configure(baudrate=20_000_000)
            self.dac_spi.unlock()
        else:
            print("could not configure DAC SPI")

        self.dac_cs = digitalio.DigitalInOut(DAC_CS)
        self.dac_cs.switch_to_output(value=True)

        for i in range(4):
            self.update()   # pre-read all the mux inputs

    def update(self):
        """Update the comptuer inputs. Should be called frequently"""
        self.mux_update(self.mux_count)
        self.mux_read(self.mux_count)
        self.mux_count = (self.mux_count + 1) % 4
        
    @property
    def knob_main(self):
        """Main knob position, raw 0-65535"""
        return self.analog[4]
    @property
    def knob_x(self):
        """X-knob position, raw 0-65535"""
        return self.analog[5]
    @property
    def knob_y(self):
        """Y-knob position, raw 0-65535"""
        return self.analog[6]
    @property
    def switch(self):
        """Switch position, raw 0-65535"""
        return self.analog[7]

    @property
    def audio_1_in(self):
        """Uninverted Audio 1 input."""
        return 65535 - self.audio_1_in_read.value

    @property
    def audio_2_in(self):
        """Uninverted Audio 2 input."""
        return 65535 - self.audio_2_in_read.value

    @property
    def cv_1_in(self):
        """Uninverted CV 1 input""" 
        return 65535 - self.analog[2]
    @property
    def cv_2_in(self):
        """Uninverted CV 1 input""" 
        return 65535 - self.analog[3]

    @property
    def pulse_1_in(self):
        """Pulse 1 Input - True if high, False if low."""
        return not self.pulse_1_in_read.value

    @property
    def pulse_2_in(self):
        """Pulse 2 Input - True if high, False if low."""
        return not self.pulse_2_in_read.value

    @property
    def pulse_1_out(self):
        """Pulse 1 Output state (True for high/active)."""
        return not self._pulse_1_out_pin.value
    
    @pulse_1_out.setter
    def pulse_1_out(self, value):
        """Sets Pulse 1 Output state (True = active)."""
        self._pulse_1_out_pin.value = not bool(value)

    @property
    def pulse_2_out(self):
        """Pulse 2 Output state (True for high/active)."""
        return not self._pulse_2_out_pin.value
    
    @pulse_2_out.setter
    def pulse_2_out(self, value):
        """Sets Pulse 2 Output state (True = active)."""
        self._pulse_2_out_pin.value = not bool(value)

    @property
    def audio_1_out(self):
        """DAC Audio 1 Output value (Raw 16-bit input)."""
        return self._audio_1_out_value

    @audio_1_out.setter
    def audio_1_out(self, value):
        """Sets DAC Audio 1 Output. Accepts 16-bit (0-65535), scales to 12-bit (0-4095)."""
        # Store the raw 16-bit input value
        self._audio_1_out_value = value
        
        # Write to DAC (inverted: 4095 - dac_value)
        self.dac_write(0, 4095 - (int(value) // 16)) # Channel 0 = DAC A

    @property
    def audio_2_out(self):
        """DAC Audio 2 Output value (Raw 16-bit input)."""
        return self._audio_2_out_value

    @audio_2_out.setter
    def audio_2_out(self, value):
        """Sets DAC Audio 2 Output. Accepts 16-bit (0-65535), scales to 12-bit (0-4095)."""
        # Store the raw 16-bit input value
        self._audio_2_out_value = value
        
        # Write to DAC (inverted: 4095 - dac_value)
        self.dac_write(1, 4095 - (int(value) // 16)) # Channel 1 = DAC B

    @property
    def cv_1_out(self):
        # FIX: Return logical value (0-65535) not the inverted duty_cycle
        return 65535 - self.cv_1_pwm.duty_cycle
    @cv_1_out.setter
    def cv_1_out(self,val):
        # FIX: Set inverted duty_cycle
        self.cv_1_pwm.duty_cycle = 65535 - min(max(int(val), 0), 65535)

    @property
    def cv_2_out(self):
        # FIX: Return logical value (0-65535) not the inverted duty_cycle
        return 65535 - self.cv_2_pwm.duty_cycle
    @cv_2_out.setter
    def cv_2_out(self,val):
        # FIX: Set inverted duty_cycle
        self.cv_2_pwm.duty_cycle = 65535 - min(max(int(val), 0), 65535)

    def mux_update(self, num):
        """Update the mux channel, used by ``update()``"""
        self.mux_A.value = (num >> 0) & 1
        self.mux_B.value = (num >> 1) & 1

    def mux_read(self, num):
        """Read into the ``analog`` list new values for
        the given mux channel, used by ``update()``"""
        s = self.analog_smooth_amount
        if num == 0:
            self.analog[4] = int(s*self.analog[4] + (1-s)*self.analog_mux1.value)  # main knob
            self.analog[2] = int(s*self.analog[2] + (1-s)*self.analog_mux2.value)  # CV 1 (inverted)
        elif num == 1:
            self.analog[5] = int(s*self.analog[5] + (1-s)*self.analog_mux1.value)  # X knob
            self.analog[3] = int(s*self.analog[3] + (1-s)*self.analog_mux2.value)  # CV 2 (inverted)
        elif num == 2:
            self.analog[6] = int(s*self.analog[6] + (1-s)*self.analog_mux1.value)  # Y knob
            self.analog[2] = int(s*self.analog[2] + (1-s)*self.analog_mux2.value)  # CV 1 (inverted)
        elif num == 3:
            self.analog[7] = int(s*self.analog[7] + (1-s)*self.analog_mux1.value)  # Switch
            self.analog[3] = int(s*self.analog[3] + (1-s)*self.analog_mux2.value)  # CV 2 (inverted)        

    def dac_write(self, channel, value):
        """
        Writes a 12-bit value to the MCP4822 DAC on the specified channel.
        """
        # DAC configuration data: Control bits (Gain/Buffer/Shutdown) + 12-bit data
        if channel == 0:
            DAC_data = DAC_config_chan_A_gain | (value & 0xFFF)
        else:
            DAC_data = DAC_config_chan_B_gain | (value & 0xFFF)

        # Pack the 16 bits (4 control + 12 data) into two bytes
        data_to_send = bytes( (DAC_data >> 8, DAC_data & 0xFF) )

        # Must acquire SPI lock, lower CS, write, raise CS, then release lock
        if self.dac_spi.try_lock():
            try:
                self.dac_cs.value = False  # Chip Select LOW to start transfer
                self.dac_spi.write(data_to_send)
                self.dac_cs.value = True   # Chip Select HIGH to finish transfer
            finally:
                self.dac_spi.unlock()


    def pulse_outs_to_audio(self, sample_rate=22050, voice_count=5, channel_count=2):
        """Convert the pulse outs to play PWM audio """
        if hasattr(self, '_pulse_1_out_pin'):  # release in-use pins
            self._pulse_1_out_pin.deinit()
            self._pulse_2_out_pin.deinit()
        if hasattr(self, 'audio'):
            self.audio.deinit()
            
        self.audio = audiopwmio.PWMAudioOut(left_channel=PULSE_1_RAW_OUT,
                                             right_channel=PULSE_2_RAW_OUT)
        self.mixer = audiomixer.Mixer(voice_count=voice_count,
                                       sample_rate=sample_rate,
                                       channel_count=channel_count,
                                       bits_per_sample=16,
                                       samples_signed=True,
                                       buffer_size=2048
                                       )
        self.audio.play(self.mixer)  # attach mixer to audio playback