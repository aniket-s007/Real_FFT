import math
import cmath
import numpy as np

class Hardware_RFFT_Golden_Model:
    def __init__(self, N):
        if (N & (N - 1)) != 0 or N < 2:
            raise ValueError("N must be a power of 2")
        self.N = N

    def rotator(self, real_in, imag_in, phi):
        """Standard hardware complex multiplier"""
        angle = -2.0 * math.pi * phi / self.N
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        
        real_out = real_in * cos_val - imag_in * sin_val
        imag_out = imag_in * cos_val + real_in * sin_val
        return real_out, imag_out

    def cfft_separated(self, r, i_arr, step, base_bin, bin_step):
        """
        Models the standard complex DIF FFT stages, but explicitly 
        forces separated real and imaginary datapath rails.
        """
        L = len(r)
        # Hardware base case: wire passes straight through
        if L == 1:
            return r, i_arr, [base_bin]

        half = L // 2
        
        # 1. Separated real-adder butterflies (No complex math here)
        top_r = [r[k] + r[k+half] for k in range(half)]
        bot_r = [r[k] - r[k+half] for k in range(half)]
        
        top_i = [i_arr[k] + i_arr[k+half] for k in range(half)]
        bot_i = [i_arr[k] - i_arr[k+half] for k in range(half)]

        # 2. Rotators applied strictly to the bottom half
        bot_r_rot = [0.0] * half
        bot_i_rot = [0.0] * half
        for k in range(half):
            res_r, res_i = self.rotator(bot_r[k], bot_i[k], k * step)
            bot_r_rot[k] = res_r
            bot_i_rot[k] = res_i

        # 3. Recursive pipeline stages (simulating hardware routing)
        out_top_r, out_top_i, top_bins = self.cfft_separated(top_r, top_i, step * 2, base_bin, bin_step * 2)
        out_bot_r, out_bot_i, bot_bins = self.cfft_separated(bot_r_rot, bot_i_rot, step * 2, base_bin + bin_step, bin_step * 2)
        
        return out_top_r + out_bot_r, out_top_i + out_bot_i, top_bins + bot_bins

    def rfft_recursive(self, x, step=1, base_bin=0, bin_step=1):
        """
        The core Garrido architecture logic: exploits real inputs to 
        skip redundant operations using Eq 7.
        """
        L = len(x)
        if L == 1:
            return x, [0.0], [base_bin]
        if L == 2:
            return [x[0] + x[1], x[0] - x[1]], [0.0, 0.0], [base_bin, base_bin + bin_step]

        half = L // 2
        
        # 1. Purely Real Butterflies (Stage 1 / Top halves)
        top = [x[k] + x[k+half] for k in range(half)]
        bot = [x[k] - x[k+half] for k in range(half)]

        # 2. Top path remains real, routes to next stage
        even_r, even_i, even_bins = self.rfft_recursive(top, step * 2, base_bin, bin_step * 2)

        # 3. Bottom path: Garrido Eq. 7 Complex Split
        q = half // 2
        c_r = [0.0] * q
        c_i = [0.0] * q
        for k in range(q):
            # Notice the explicit negative sign on the imaginary input per Eq 7
            r, img = self.rotator(bot[k], -bot[k+q], k * step)
            c_r[k] = r
            c_i[k] = img

        # 4. Once split, the bottom path routes into the separated complex pipeline
        odd_r, odd_i, odd_bins = self.cfft_separated(c_r, c_i, step * 4, base_bin + bin_step, bin_step * 4)

        return even_r + odd_r, even_i + odd_i, even_bins + odd_bins

    def compute(self, x):
        """
        Wrapper that runs the model and mathematically unscrambles 
        the hardware wire outputs into sequential frequency bins (0 to N/2).
        """
        if len(x) != self.N:
            raise ValueError(f"Input length {len(x)} does not match initialized N={self.N}")

        raw_r, raw_i, raw_bins = self.rfft_recursive(x)
        
        out = [0j] * (self.N // 2 + 1)
        
        for r, img, b in zip(raw_r, raw_i, raw_bins):
            if b <= self.N // 2:
                # Direct bins
                out[b] = complex(r, img)
            else:
                # Conjugate symmetry for hardware bins mapped > N/2
                out[self.N - b] = complex(r, -img)
                
        return np.array(out)

# ==========================================
# Automated Scaling Test Bench
# ==========================================
if __name__ == "__main__":
    # Our target scaling sizes, exactly as requested
    target_sizes = [16, 32, 64, 128, 256, 512, 1024, 2048, 4096]
    
    print(f"{'N':<6} | {'Status':<15} | {'Max Error':<15}")
    print("-" * 45)
    
    for N in target_sizes:
        np.random.seed(42)
        x_test = np.random.rand(N).tolist()
        
        # 1. Initialize our generalized hardware model
        hardware_model = Hardware_RFFT_Golden_Model(N)
        X_custom = hardware_model.compute(x_test)
        
        # 2. Run standard Numpy RFFT
        X_numpy = np.fft.rfft(x_test)
        
        # 3. Verify
        try:
            np.testing.assert_allclose(X_custom, X_numpy, atol=1e-9)
            max_err = np.max(np.abs(X_custom - X_numpy))
            print(f"{N:<6} | SUCCESS         | {max_err:.4e}")
        except AssertionError as e:
            print(f"{N:<6} | FAILED          | CHECK LOGS")
            print(e)
            break