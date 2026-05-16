import struct

def create_midi_file(filename):
    ticks_per_beat = 480
    header = struct.pack(">4sIHHH", b"MThd", 6, 0, 1, ticks_per_beat)
    
    track_events = []
    
    def write_vlq(delta):
        res = []
        if delta == 0:
            return b"\x00"
        while delta > 0:
            res.append(delta & 0x7f)
            delta >>= 7
        out = []
        for i in range(len(res)-1, 0, -1):
            out.append(res[i] | 0x80)
        out.append(res[0])
        return bytes(out)

    current_delta = 0

    def add_event(delta, event):
        nonlocal current_delta
        current_delta += delta
        if event:
            track_events.append(write_vlq(current_delta))
            track_events.append(event)
            current_delta = 0

    # 1. Setup
    add_event(0, b"\xB0\x01\x7F") # Strength Max
    add_event(0, b"\xB0\x46\x00") # Geometry Min
    add_event(0, b"\xB0\x4A\x40") # Brightness Mid
    add_event(0, b"\xB0\x47\x40") # Damping Mid

    # 2. Chromatic Scale
    notes = [48, 50, 52, 53, 55, 57, 59, 60, 62, 64, 65, 67, 69, 71, 72]
    for n in notes:
        add_event(480, bytes([0x90, n, 100]))
        add_event(240, bytes([0x80, n, 0]))

    # 3. Resonator Sweep
    add_event(480, bytes([0x90, 60, 100]))
    for i in range(128):
        add_event(10, bytes([0xB0, 70, i]))
        add_event(0, bytes([0xB0, 74, 127-i]))
    add_event(480, bytes([0x80, 60, 0]))

    # 4. Exciter Morph
    for i in range(16):
        n = 60 + (i % 12)
        strike_lvl = max(0, 127 - i*8)
        blow_lvl = min(127, i*8)
        add_event(0, bytes([0xB0, 12, strike_lvl]))
        add_event(0, bytes([0xB0, 15, blow_lvl]))
        add_event(240, bytes([0x90, n, 100]))
        add_event(240, bytes([0x80, n, 0]))

    # 5. Spatial Sweep
    add_event(480, bytes([0x90, 48, 100]))
    for i in range(64):
        add_event(20, bytes([0xB0, 21, i*2]))
    add_event(480, bytes([0x80, 48, 0]))

    # End Track
    add_event(480, b"\xFF\x2F\x00")
    
    track_data = b"".join(track_events)
    track_chunk = struct.pack(">4sI", b"MTrk", len(track_data)) + track_data
    
    with open(filename, "wb") as f:
        f.write(header)
        f.write(track_chunk)

if __name__ == "__main__":
    create_midi_file("modal_test.mid")
    print("Generated modal_test.mid")
