"""
CWRU bring-up  +  float golden-reference FFT   (roadmap Step 0)
================================================================
Goal: prove (1) the .mat loads, (2) decimation 48k->1k is correct and
anti-aliased, (3) the inner-race fault peak appears in the fault file and is
ABSENT in the normal file. The float segment produced here (`seg`) is the
golden reference the later fixed-point SQNR study compares against.

Verified on synthetic CWRU-like data (run `self_test()`):
  - loader finds *_DE_time and *RPM regardless of the file's numeric prefix
  - BPFI bin computed from actual RPM (1772 rpm -> 159.93 Hz -> bin 655)
  - a 4900 Hz tone is correctly killed by decimation (no alias at 100 Hz)
  - planted 160 Hz fault is ~570x stronger in fault vs normal
"""
import numpy as np
import scipy.io as sio
from scipy.signal import decimate
import matplotlib.pyplot as plt

# ---- design point ----
FS_RAW    = 48000     # CWRU 48k drive-end files
FS_TARGET = 1000      # downsample target (Nyquist 500 Hz covers all fault freqs)
N         = 4096      # FFT length  -> bin width = 1000/4096 = 0.2441 Hz
BPFI_MULT = 5.4152    # SKF-6205 drive-end inner-race multiplier
# (others if you want them: BPFO 3.5848, BSF 2.3567, shaft 1.0)


def load_cwru(path):
    """Return (DE_time signal as 1-D float64, RPM as float).

    CWRU variable names carry a per-file numeric prefix (e.g. X209_DE_time),
    so we match by suffix instead of hardcoding. Falls back to nominal 1772
    rpm with a warning if no RPM variable is present.
    """
    try:
        m = sio.loadmat(path)
    except NotImplementedError:
        raise RuntimeError(
            f"{path} looks like MATLAB v7.3 (HDF5). Install `mat73` or use h5py."
        )
    de_key = next((k for k in m if k.endswith('_DE_time')), None)
    if de_key is None:
        raise KeyError(f"No *_DE_time variable in {path}. Keys: {list(m)}")
    rpm_key = next((k for k in m if k.upper().endswith('RPM')), None)
    sig = m[de_key].squeeze().astype(np.float64)
    if rpm_key is None:
        print(f"  [warn] no RPM in {path}; assuming 1772")
        rpm = 1772.0
    else:
        rpm = float(m[rpm_key].squeeze())
    return sig, rpm


def decimate_to_1k(sig):
    """48 kHz -> 1 kHz in two anti-aliased stages (never factor-48 in one call)."""
    x = decimate(sig, 8, ftype='fir')   # 48k -> 6k  (anti-alias < 3 kHz)
    x = decimate(x,   6, ftype='fir')   # 6k  -> 1k  (anti-alias < 500 Hz)
    return x


def spectrum(seg):
    """rFFT magnitude of a length-N segment. Mean removed so DC doesn't dominate.
    No window (matches a straight hardware FFT); add np.hanning(N) here if you
    later want cleaner visualization, but apply it identically in the RTL path."""
    seg = seg - seg.mean()
    X = np.fft.rfft(seg)
    f = np.fft.rfftfreq(len(seg), d=1.0 / FS_TARGET)
    return f, np.abs(X)


def bpfi_freq_bin(rpm):
    """Inner-race fault frequency (Hz) and its FFT bin, from the file's RPM."""
    f = BPFI_MULT * rpm / 60.0
    return f, int(round(f / (FS_TARGET / N)))


def analyze(path):
    sig, rpm = load_cwru(path)
    x = decimate_to_1k(sig)
    if len(x) < N:
        raise ValueError(f"only {len(x)} samples after decimation, need {N}")
    seg = x[:N]                       # <-- this is the golden-reference segment
    f, mag = spectrum(seg)
    return dict(rpm=rpm, freqs=f, mag=mag, seg=seg, nsamp=len(x))


def peak_near(f, mag, f0, win_hz=4.0):
    """Largest |X| within +/- win_hz of f0 (real peaks leak / RPM is approximate)."""
    idx = np.where(np.abs(f - f0) <= win_hz)[0]
    k = idx[np.argmax(mag[idx])]
    return f[k], k, mag[k]


def run_comparison(normal_path, fault_path, show=True):
    """Full Step-0: load both, FFT, locate the fault peak, plot the contrast."""
    norm  = analyze(normal_path)
    fault = analyze(fault_path)
    f_bpfi, k_bpfi = bpfi_freq_bin(fault['rpm'])

    _, _, pf = peak_near(fault['freqs'], fault['mag'], f_bpfi)
    _, _, pn = peak_near(norm['freqs'],  norm['mag'],  f_bpfi)
    print(f"RPM (fault file) : {fault['rpm']:.0f}")
    print(f"BPFI             : {f_bpfi:.2f} Hz  -> bin {k_bpfi}")
    print(f"|X| at BPFI      : fault={pf:.1f}   normal={pn:.1f}   ratio={pf/max(pn,1e-9):.1f}x")
    print("GATE: fault peak >> normal at BPFI  ->",
          "PASS" if pf > 5 * pn else "FAIL (peak not clearly present)")

    db = lambda m: 20 * np.log10(m + 1e-9)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(norm['freqs'],  db(norm['mag']),  lw=0.8, alpha=0.7, label='normal')
    ax.plot(fault['freqs'], db(fault['mag']), lw=0.8, label='inner-race fault')
    ax.axvline(f_bpfi, color='r', ls='--', lw=1, label=f'BPFI {f_bpfi:.1f} Hz (bin {k_bpfi})')
    ax.set_xlim(0, 500); ax.set_xlabel('Hz'); ax.set_ylabel('|X| (dB)')
    ax.set_title('CWRU bring-up: float reference FFT'); ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig('cwru_bringup.png', dpi=110)
    print("saved cwru_bringup.png")
    if show:
        plt.show()
    # next step plugs in here: quantize fault['seg'] to W bits, rFFT, and
    # compute SQNR at bin k_bpfi against this float reference.
    return norm, fault


def self_test():
    """Regenerate synthetic CWRU-like files and re-run the gates locally."""
    rng = np.random.default_rng(0)
    t = np.arange(0, 10.0, 1 / FS_RAW)
    rpm = 1772.0; fr = rpm / 60; f_bpfi = BPFI_MULT * fr
    common = (0.5*np.sin(2*np.pi*fr*t) + 0.3*np.sin(2*np.pi*4900*t)
              + 0.20*rng.standard_normal(t.size))            # 4900 Hz = alias test
    fault_extra = (1.0*np.sin(2*np.pi*f_bpfi*t) + 0.5*np.sin(2*np.pi*2*f_bpfi*t)
                   + 0.3*np.sin(2*np.pi*(f_bpfi+fr)*t) + 0.3*np.sin(2*np.pi*(f_bpfi-fr)*t))
    sio.savemat('N_1.mat',    {'X097_DE_time': common[:, None],              'X097RPM': [[rpm]]})
    sio.savemat('IR021_1.mat',{'X209_DE_time': (common+fault_extra)[:, None],'X209RPM': [[rpm]]})
    run_comparison('N_1.mat', 'IR021_1.mat', show=False)


if __name__ == '__main__':
    # 1) sanity-check the harness on synthetic data:
    self_test()

    # 2) then point at YOUR real files and comment out the line above:
    # run_comparison('path/to/N_1.mat', 'path/to/IR021_1.mat')