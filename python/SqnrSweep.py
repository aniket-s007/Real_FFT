"""
SQNR sweep: minimum FFT wordlength for bearing fault detection.
Both Normal_1.mat and IR021_1.mat must be next to this script.
"""
import numpy as np
import scipy.io as sio
from scipy.signal import decimate, butter, filtfilt, hilbert
import matplotlib.pyplot as plt
import os

HERE      = os.path.dirname(os.path.abspath(__file__))  # folder this script lives in
FS_TARGET = 1000     # downsample target in Hz
N         = 4096     # FFT length -> 0.2441 Hz per bin
BPFI_MULT = 5.4152   # SKF-6205 inner-race fault-frequency multiplier
BAND      = (4000, 6000)  # resonance band chosen by envelope_detect.py
THRESHOLD = 5.0      # broken/healthy ratio required to call it "detected"


def load_envelope(filename):
    """Load a CWRU .mat file and return a normalised 4096-pt envelope segment + RPM."""
    path = os.path.join(HERE, filename)                            # full path to the file
    m    = sio.loadmat(path)                                       # read the .mat file
    de   = next(k for k in m if k.endswith('_DE_time'))           # find DE vibration variable
    rpmk = next((k for k in m if k.upper().endswith('RPM')), None)# find RPM variable if present
    sig  = m[de].squeeze().astype(float)                           # extract signal as 1-D float64
    rpm  = float(m[rpmk].squeeze()) if rpmk else 1772.0           # read RPM, or use nominal
    fs   = 48000 if len(sig) > 200000 else 12000                  # detect sample rate from length
    b, a = butter(4, [BAND[0]/(fs/2), BAND[1]/(fs/2)], 'band')   # bandpass around resonance
    env  = np.abs(hilbert(filtfilt(b, a, sig)))                    # Hilbert envelope of ringing
    env  = env - env.mean()                                        # remove DC from envelope
    env  = decimate(env, 8, ftype='fir')                           # 48k -> 6k (stage 1)
    env  = decimate(env, 6, ftype='fir')                           # 6k  -> 1k (stage 2)
    seg  = env[:N]                                                 # first N samples
    seg  = seg / np.max(np.abs(seg))                               # normalise to [-1, 1)
    return seg, rpm


def fft_fixed(x, W):
    """W-bit fixed-point radix-2 DIT FFT. Quantises input, twiddles, and every butterfly output."""
    N      = len(x)                                     # transform length
    levels = N.bit_length() - 1                        # number of stages = log2(N)
    lsb    = 2.0 ** -(W - 1)                           # value of 1 LSB in W-bit signed format

    def q(z):
        """Round complex number to nearest W-bit fixed-point value."""
        real_q = np.round(z.real / lsb) * lsb          # quantise real part
        imag_q = np.round(z.imag / lsb) * lsb          # quantise imaginary part
        return real_q + 1j * imag_q                     # return quantised complex value

    rev = np.array([int(f'{i:0{levels}b}'[::-1], 2) for i in range(N)])  # bit-reversal index
    X   = q(x[rev].astype(complex))                    # bit-reverse input and quantise it

    for s in range(1, levels + 1):                     # loop over each butterfly stage
        m    = 1 << s                                   # butterfly group size at this stage
        h    = m >> 1                                   # half-group index (upper vs lower)
        tw   = q(np.exp(-2j * np.pi * np.arange(h) / m))  # twiddle factors, quantised
        G    = X.reshape(-1, m)                         # split into butterfly groups
        top  = G[:, :h].copy()                         # upper half of each butterfly
        bot  = G[:, h:] * tw                           # lower half multiplied by twiddle
        G[:, :h] = q((top + bot) / 2)                  # butterfly sum, /2 prevents overflow
        G[:, h:] = q((top - bot) / 2)                  # butterfly difference, /2 prevents overflow
        X    = G.reshape(-1)                            # flatten for next stage

    return X                                            # output is DFT/N (same scale as np.fft.fft/N)


def run_sweep(seg_b, seg_h, binf, threshold=THRESHOLD):
    """Sweep W=4..16, compute broken/healthy ratio at fault bin, find minimum W."""
    Ws     = list(range(4, 17))                         # wordlengths to test
    ratios = []                                         # ratio result for each W

    for W in Ws:
        lsb    = 2.0 ** -(W - 1)                       # quantisation step at this wordlength
        rb     = np.abs(fft_fixed(seg_b, W))           # fixed-point FFT magnitude of broken
        rh     = np.abs(fft_fixed(seg_h, W))           # fixed-point FFT magnitude of healthy
        b_peak = rb[binf]                              # broken magnitude at fault bin
        h_val  = rh[binf]                              # healthy magnitude at fault bin
        floor  = max(h_val, lsb)                       # use lsb as floor when healthy is zero
        ratio  = b_peak / floor                        # broken-to-healthy ratio at fault bin
        ratios.append(ratio)
        print(f"  W={W:2d}  broken={b_peak:.3e}  healthy={h_val:.3e}  ratio={ratio:6.1f}x")

    minW = next((W for W, r in zip(Ws, ratios) if r >= threshold), None)  # first W above threshold
    return Ws, ratios, minW


def main():
    """Load both files, build envelopes, run sweep, plot result."""
    print("loading files...")
    seg_b, rpm = load_envelope('IR021_1.mat')          # broken bearing envelope segment
    seg_h, _   = load_envelope('Normal_1.mat')         # healthy bearing envelope segment
    binf       = round(BPFI_MULT * rpm / 60 / (FS_TARGET / N))  # fault bin index

    print(f"RPM={rpm:.0f}  BPFI={BPFI_MULT*rpm/60:.2f} Hz  fault bin={binf}\n")

    Ws, ratios, minW = run_sweep(seg_b, seg_h, binf)   # run the sweep

    print(f"\n>> MINIMUM WORDLENGTH for fault detection = {minW} bits")

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(Ws, ratios, 'o-', label='broken / healthy at fault bin')        # ratio curve
    ax.axhline(THRESHOLD, color='r', ls='--', label=f'threshold ({THRESHOLD:.0f}x)')  # threshold line
    if minW:
        ax.axvline(minW, color='g', ls=':', label=f'min = {minW} bits')     # minimum W marker
    ax.set_yscale('log')                                # log scale so low values are visible
    ax.set_xlabel('FFT wordlength (bits)')              # x axis label
    ax.set_ylabel('broken / healthy ratio at fault bin')# y axis label
    ax.set_title('Minimum wordlength to detect bearing fault')  # title
    ax.legend()                                         # legend
    fig.tight_layout()
    out = os.path.join(HERE, 'sqnr_sweep.png')         # output path
    fig.savefig(out, dpi=110)                           # save figure
    print(f"saved {out}")
    plt.show()                                          # display


if __name__ == '__main__':
    main()                                              # only runs when executed directly