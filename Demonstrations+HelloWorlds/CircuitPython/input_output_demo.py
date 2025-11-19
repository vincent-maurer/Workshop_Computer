import time
from mtm_computer import Computer, gamma_correct

# Setup
comp = Computer()

last_log_time = 0.0

def rect_scale(raw: int) -> int:
    """Scales positive values for led"""
    return (raw - 32768) * 2 if raw > 32768 else 0

while True:
    # 1. Update hardware
    comp.update()
    current_time = time.monotonic()

    # 2. Read raw inputs
    # Corrected property names to match the library
    a1_raw = comp.audio_1_in 
    a2_raw = comp.audio_2_in
    cv1_raw = comp.cv_1_in
    cv2_raw = comp.cv_2_in
    main_knob = comp.knob_main
    x_knob = comp.knob_x
    y_knob = comp.knob_y
    p1_state = comp.pulse_1_in
    p2_state = comp.pulse_2_in
    switch_raw = comp.switch

    # --- Processing and LED/CV Assignment ---

    # Initialize all lights off.
    led_0_bright, led_1_bright, led_2_bright, led_3_bright, led_4_bright, led_5_bright = 0, 0, 0, 0, 0, 0

    # Set default/reset states for outputs
    # DAC_MID_VAL_16BIT = 32768 (0V for 16-bit input to new setter)
    DAC_MID_VAL_16BIT = 32768
    audio_out_1_val = DAC_MID_VAL_16BIT
    audio_out_2_val = DAC_MID_VAL_16BIT
    pulse_out_1_state = False
    pulse_out_2_state = False

    # Determine Switch State and apply logic
    # SWITCH UP
    if switch_raw > 50000:
        comp.normalisation_probe_out = False # turns off probe

        # All Outputs mirror Inputs
        # CV Output Mirroring
        comp.cv_1_out = cv1_raw
        comp.cv_2_out = cv2_raw

        # Audio Output Mirroring (16-bit value is now passed directly)
        audio_out_1_val = a1_raw
        audio_out_2_val = a2_raw

        # Pulse Output Mirroring
        pulse_out_1_state = p1_state
        pulse_out_2_state = p2_state

        # LED Visualization
        led_0_bright = rect_scale(a1_raw)
        led_1_bright = rect_scale(a2_raw)
        led_2_bright = rect_scale(cv1_raw)
        led_3_bright = rect_scale(cv2_raw)
        led_4_bright = 65535 if p1_state else 0
        led_5_bright = 65535 if p2_state else 0

    # SWITCH DOWN
    elif switch_raw < 15000:
        print("SWITCH DOWN")


    # Assign DAC Audio Outs (Corrected property names)
    comp.audio_1_out = audio_out_1_val
    comp.audio_2_out = audio_out_2_val

    # Assign Pulse Outs
    comp.pulse_1_out = pulse_out_1_state
    comp.pulse_2_out = pulse_out_2_state

    # Assign LEDs
    comp.leds[0].duty_cycle = gamma_correct(led_0_bright)
    comp.leds[1].duty_cycle = gamma_correct(led_1_bright)
    comp.leds[2].duty_cycle = gamma_correct(led_2_bright)
    comp.leds[3].duty_cycle = gamma_correct(led_3_bright)
    comp.leds[4].duty_cycle = gamma_correct(led_4_bright)
    comp.leds[5].duty_cycle = gamma_correct(led_5_bright)

    # Log current status in tuple format

    if current_time - last_log_time >= 0.1:
        last_log_time = current_time
        print((
            a1_raw, a2_raw, cv1_raw, cv2_raw,
            main_knob, x_knob, y_knob, switch_raw,
            # Logging Pulse states for clarity
            32768 + 32767 * int(p1_state), 32768 + 32767 * int(p2_state)
        ))