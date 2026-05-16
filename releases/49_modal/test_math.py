def sample_phase_inc(timbre):
    # original
    # 65536 * (32/24) * 2^((72*timbre - 29)/12)
    t = timbre / 32767.0
    st = 72.0 * t - 29.0
    orig = 65536.0 * (32.0/24.0) * (2.0 ** (st / 12.0))
    return orig

def gran_phase_inc(timbre):
    t = timbre / 32767.0
    st = 72.0 * t - 60.0
    orig = 131072.0 * (32.0/24.0) * (2.0 ** (st / 12.0))
    return orig

print("Sample Phase Inc (timbre=0.5):", sample_phase_inc(16384))
print("Gran Phase Inc (timbre=0.5):", gran_phase_inc(16384))

