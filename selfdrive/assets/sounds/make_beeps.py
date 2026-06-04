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


def click(freq, duration_seconds=0.12, ring_tc=800.0, tick_amt=0.5, tick_tc=60.0, tick_lp=3500.0, seed=0):
    # percussive woodblock-style click for cruise set-speed step feedback
    n_total = int(sr * duration_seconds)
    t = np.arange(n_total)
    rng = np.random.default_rng(seed)  # seeded so the asset is reproducible
    ring = np.sin(2 * np.pi * freq * t / sr) * np.exp(-t / ring_tc)
    tick = rng.standard_normal(n_total) * np.exp(-t / tick_tc)
    pad = 512
    tp = np.concatenate([np.zeros(pad), tick, np.zeros(pad)])
    f = np.fft.rfftfreq(len(tp), 1 / sr)
    w = 2000.0  # transition band width
    H = np.clip(0.5 * (1 + np.cos(np.pi * (f - tick_lp) / w)), 0.0, 1.0)
    H[f <= tick_lp] = 1.0
    H[f >= tick_lp + w] = 0.0
    tick = np.fft.irfft(np.fft.rfft(tp) * H, n=len(tp))[pad:pad + n_total]
    signal = ring + tick_amt * tick
    signal[:24] *= np.linspace(0.0, 1.0, 24)  # ~0.5 ms ramp to avoid a DC click-pop onset
    signal[-240:] *= np.cos(np.linspace(0.0, np.pi / 2, 240))  # 5 ms fade-out so the tail ends at zero
    return signal / np.max(np.abs(signal)) * max_int16

wavfile.write("cruise_step_up.wav", sr, click(2000.0, ring_tc=1800.0).astype(np.int16))  # brighter pitch -> step up
wavfile.write("cruise_step_down.wav", sr, click(1500.0, ring_tc=2900.0).astype(np.int16))  # darker pitch -> step down
