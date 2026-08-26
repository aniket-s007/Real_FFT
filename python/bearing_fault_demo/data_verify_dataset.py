"""
Bearing fault detection by ENVELOPE ANALYSIS  (the standard method).
Files expected next to this script: Normal_1.mat, IR021_1.mat. Just press Run.

A cracked bearing makes sharp taps that 'ring' a high-frequency resonance; the
fault's rhythm is hidden in how that ringing pulses. This script:
  1) shows the plain FFT (fault peak is weak -- ringing was filtered out), then
  2) demodulates the ringing (envelope) and FFTs that -> strong, clean peak.
The chip still computes a plain real FFT; here it just runs on the envelope.
"""
import os, sys
import numpy as np
import scipy.io as sio
from scipy.signal import decimate, butter, filtfilt, hilbert
from scipy.stats import kurtosis
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FS_RAW_OVERRIDE = None
FS_TARGET, N, BPFI_MULT = 1000, 4096, 5.4152
CANDIDATE_BANDS = [(1000,3000),(2000,4000),(3000,5000),(4000,6000),(2000,5000)]


def load_cwru(path):
    m = sio.loadmat(path)
    keys = [k for k in m if not k.startswith('__')]
    de = next((k for k in keys if k.endswith('_DE_time')), None) \
         or next((k for k in keys if 'de' in k.lower() and 'time' in k.lower()), None) \
         or (max(keys, key=lambda k: np.asarray(m[k]).size) if keys else None)
    sig = np.asarray(m[de]).squeeze().astype(np.float64)
    rpm_k = next((k for k in keys if k.upper().endswith('RPM')), None)
    rpm = float(np.asarray(m[rpm_k]).squeeze()) if rpm_k else 1772.0
    return sig, rpm

def detect_fs(n):
    return FS_RAW_OVERRIDE or (48000 if n > 200000 else 12000)

def _stages(f):
    s=[]
    for p in (8,7,6,5,4,3,2):
        while f%p==0 and f>1: s.append(p); f//=p
    if f>1: s.append(f)
    return s

def to_1k(x, fs):
    for s in _stages(fs//FS_TARGET): x = decimate(x, s, ftype='fir')
    return x

def fft_mag(seg):
    seg = seg - seg.mean()
    return np.fft.rfftfreq(N,1/FS_TARGET), np.abs(np.fft.rfft(seg[:N]))

def bpfi(rpm): return BPFI_MULT*rpm/60.0

def peak_at(f, mag, f0, win=2.0):
    return mag[np.where(np.abs(f-f0)<=win)[0]].max()

def raw_spectrum(sig, fs):
    return fft_mag(to_1k(sig, fs)[:N])

def best_band(sig, fs):
    """Most impulsive band = where the impacts ring (kurtogram-lite)."""
    best, bk = CANDIDATE_BANDS[-1], -np.inf
    for lo,hi in CANDIDATE_BANDS:
        if hi >= fs/2: continue
        b,a = butter(4,[lo/(fs/2),hi/(fs/2)],'band')
        k = kurtosis(filtfilt(b,a,sig))
        if k > bk: bk, best = k, (lo,hi)
    return best, bk

def envelope_spectrum(sig, fs, band):
    b,a = butter(4,[band[0]/(fs/2),band[1]/(fs/2)],'band')
    env = np.abs(hilbert(filtfilt(b,a,sig)))     # demodulate the ringing
    return fft_mag(to_1k(env - env.mean(), fs)[:N])


def main(normal_file, fault_file):
    for p in (normal_file, fault_file):
        if not os.path.exists(p):
            print(f"FILE NOT FOUND: {p}"); return
    sig_n, rpm_n = load_cwru(normal_file)
    sig_f, rpm_f = load_cwru(fault_file)
    fs_n, fs_f = detect_fs(len(sig_n)), detect_fs(len(sig_f))
    f0 = bpfi(rpm_f)
    print(f"healthy: {len(sig_n)} samp -> {fs_n} Hz, RPM {rpm_n:.0f}")
    print(f"broken : {len(sig_f)} samp -> {fs_f} Hz, RPM {rpm_f:.0f}")
    print(f"inner-race fault frequency (BPFI): {f0:.2f} Hz\n")

    # (1) plain FFT
    fr_f, mr_f = raw_spectrum(sig_f, fs_f)
    fr_n, mr_n = raw_spectrum(sig_n, fs_n)
    rr = peak_at(fr_f, mr_f, f0) / max(peak_at(fr_n, mr_n, f0), 1e-9)
    print(f"[1] PLAIN FFT        ratio broken/healthy at BPFI = {rr:5.1f}x")

    # (2) envelope FFT
    band, kt = best_band(sig_f, fs_f)
    print(f"    chose ringing band {band} Hz (kurtosis {kt:.1f})")
    fe_f, me_f = envelope_spectrum(sig_f, fs_f, band)
    fe_n, me_n = envelope_spectrum(sig_n, fs_n, band)
    re = peak_at(fe_f, me_f, f0) / max(peak_at(fe_n, me_n, f0), 1e-9)
    print(f"[2] ENVELOPE FFT     ratio broken/healthy at BPFI = {re:5.1f}x")
    print("\n>> FAULT DETECTED" if re > 5 else "\n>> still weak; try widening CANDIDATE_BANDS",
          f"(envelope is {re/max(rr,1e-9):.0f}x better than plain FFT)")

    db = lambda m: 20*np.log10(m+1e-9)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for ax, (fn, mn, ff, mf, ttl) in zip(
        (a1, a2),
        [(fr_n, mr_n, fr_f, mr_f, f'[1] Plain FFT  (ratio {rr:.1f}x)'),
         (fe_n, me_n, fe_f, me_f, f'[2] Envelope FFT  (ratio {re:.1f}x)  <- the method')]):
        ax.plot(fn, db(mn), lw=0.8, alpha=.7, label='healthy')
        ax.plot(ff, db(mf), lw=0.8, label='broken')
        ax.axvline(f0, color='r', ls='--', lw=1, label=f'BPFI {f0:.1f} Hz')
        ax.set_xlim(0,500); ax.set_ylabel('dB'); ax.set_title(ttl); ax.legend(fontsize=8)
    a2.set_xlabel('Hz'); fig.tight_layout()
    fig.savefig(os.path.join(HERE,'envelope_result.png'), dpi=110)
    print("saved envelope_result.png"); plt.show()


if __name__ == '__main__':
    nf = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "Normal_1.mat")
    ff = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "IR021_1.mat")
    main(nf, ff)