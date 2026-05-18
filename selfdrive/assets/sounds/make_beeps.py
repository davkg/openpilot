import numpy as np
from scipy.io import wavfile


sr = 48000
max_int16 = 2**15 - 1

def harmonic_beep(freq, duration_seconds):
    n_total = int(sr * duration_seconds)

    signal = np.sin(2 * np.pi * freq * np.arange(n_total) / sr)
    x = np.arange(n_total)
    exp_scale = np.exp(-x/5.5e3)
    return max_int16 * signal * exp_scale

engage_beep = harmonic_beep(1661.219, 0.5)
wavfile.write("engage.wav", sr, engage_beep.astype(np.int16))
disengage_beep = harmonic_beep(1318.51, 0.5)
wavfile.write("disengage.wav", sr, disengage_beep.astype(np.int16))


def click(freq, duration_seconds=0.075):
    # short two-partial percussive click for cruise set-speed step feedback
    n_total = int(sr * duration_seconds)
    t = np.arange(n_total)
    tone = np.sin(2 * np.pi * freq * t / sr) + 0.35 * np.sin(2 * np.pi * 2 * freq * t / sr)
    env = np.exp(-t / 600.0)
    attack = min(96, n_total)  # ~2 ms ramp to avoid a click-pop transient
    env[:attack] *= np.linspace(0.0, 1.0, attack)
    signal = tone * env
    return signal / np.max(np.abs(signal)) * 0.7 * max_int16

wavfile.write("cruise_step_up.wav", sr, click(1175.0).astype(np.int16))
wavfile.write("cruise_step_down.wav", sr, click(880.0).astype(np.int16))
