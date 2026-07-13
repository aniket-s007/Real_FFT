import math
import numpy as np

class HardwareRFFT_Iterative:
    def __init__(self, N):
        if (N & (N - 1)) != 0 or N < 2:
            raise ValueError("N must be a power of 2")
        self.N = N
        self.stages = int(math.log2(N))
        
        # 1. Hardware Memory Allocation (Fixed Size In-Place RAM)
        self.RAM = [0.0] * N

    def rotator(self, real_in, imag_in, phi):
        """Simulates the physical complex multiplier pipeline block"""
        angle = -2.0 * math.pi * phi / self.N
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        
        r_out = real_in * cos_val - imag_in * sin_val
        i_out = imag_in * cos_val + real_in * sin_val
        return r_out, i_out

    def compute(self, x_in):
        # 2. Input Streaming
        for i in range(self.N):
            self.RAM[i] = float(x_in[i])

        # 3. Control FIFO Initialization
        # Descriptor: (Type, r_start, i_start, length, phi_step, base_bin, bin_step)
        ctrl_fifo = [('REAL', 0, 0, self.N, 1, 0, 1)]
        next_ctrl_fifo = []

        # =========================================================
        # 4. HARDWARE FSM: Descriptor-Driven AGU
        # =========================================================
        for stage in range(self.stages):
            
            # The AGU processes blocks until the FIFO for this stage is completely empty
            while len(ctrl_fifo) > 0:
                curr_block = ctrl_fifo.pop(0)
                b_type, r_start, i_start, length, phi_step, base, binstep = curr_block
                
                cycles_in_block = length // 2
                
                # -------------------------------------------------
                # DATAPATH EXECUTION (Runs uninterrupted for 'cycles_in_block')
                # -------------------------------------------------
                for k in range(cycles_in_block):
                    if b_type == 'REAL':    #block type: REAL
                        a_idx = r_start + k
                        b_idx = r_start + k + cycles_in_block
                        
                        A = self.RAM[a_idx]
                        B = self.RAM[b_idx]
                        self.RAM[a_idx] = A + B
                        self.RAM[b_idx] = A - B
                        
                    elif b_type == 'EQ7':
                        r_idx = r_start + k
                        i_idx = r_start + k + cycles_in_block
                        
                        A = self.RAM[r_idx]
                        B = -self.RAM[i_idx] # Eq.7 explicit hardware negation
                        
                        R, I = self.rotator(A, B, k * phi_step)
                        self.RAM[r_idx] = R
                        self.RAM[i_idx] = I
                        
                    elif b_type == 'CFFT':
                        r_a = r_start + k
                        r_b = r_start + k + cycles_in_block
                        i_a = i_start + k
                        i_b = i_start + k + cycles_in_block
                        
                        A_r = self.RAM[r_a]
                        B_r = self.RAM[r_b]
                        A_i = self.RAM[i_a]
                        B_i = self.RAM[i_b]
                        
                        R_top = A_r + B_r
                        I_top = A_i + B_i
                        R_bot = A_r - B_r
                        I_bot = A_i - B_i
                        
                        R_rot, I_rot = self.rotator(R_bot, I_bot, k * phi_step)
                        
                        self.RAM[r_a] = R_top
                        self.RAM[i_a] = I_top
                        self.RAM[r_b] = R_rot
                        self.RAM[i_b] = I_rot

                # -------------------------------------------------
                # CONTROL LOGIC: Schedule next state upon block completion
                # -------------------------------------------------
                if b_type == 'REAL':
                    if cycles_in_block >= 2:
                        next_ctrl_fifo.append(('REAL', r_start, 0, cycles_in_block, phi_step * 2, base, binstep * 2))
                        next_ctrl_fifo.append(('EQ7', r_start + cycles_in_block, 0, cycles_in_block, phi_step, base + binstep, binstep * 2))
                    else:
                        next_ctrl_fifo.append(('DONE_REAL', r_start, 0, 2, 0, base, binstep))
                        
                elif b_type == 'EQ7':
                    if cycles_in_block >= 2:
                        next_ctrl_fifo.append(('CFFT', r_start, r_start + cycles_in_block, cycles_in_block, phi_step * 4, base, binstep * 2))
                    else:
                        next_ctrl_fifo.append(('DONE_CFFT', r_start, r_start + 1, 2, 0, base, binstep))
                        
                elif b_type == 'CFFT':
                    if cycles_in_block >= 2:
                        next_ctrl_fifo.append(('CFFT', r_start, i_start, cycles_in_block, phi_step * 2, base, binstep * 2))
                        next_ctrl_fifo.append(('CFFT', r_start + cycles_in_block, i_start + cycles_in_block, cycles_in_block, phi_step * 2, base + binstep, binstep * 2))
                    else:
                        next_ctrl_fifo.append(('DONE_CFFT', r_start, i_start, 2, 0, base, binstep))
                        next_ctrl_fifo.append(('DONE_CFFT', r_start + 1, i_start + 1, 2, 0, base + binstep, binstep))
                        
            # End of Clock Stage: Ping-Pong the Control FIFOs
            ctrl_fifo = next_ctrl_fifo
            next_ctrl_fifo = []

        # =========================================================
        # 5. Output Extraction (Streaming out of RAM)
        # =========================================================
        out = [0j] * (self.N // 2 + 1)
        
        for blk in ctrl_fifo:
            b_type, r_start, i_start, _, _, base, binstep = blk
            
            if b_type == 'DONE_REAL':
                bins = [(base, complex(self.RAM[r_start], 0)), 
                        (base + binstep, complex(self.RAM[r_start + 1], 0))]
            elif b_type == 'DONE_CFFT':
                bins = [(base, complex(self.RAM[r_start], self.RAM[i_start]))]
            else:
                continue
                
            for b, val in bins:
                if b <= self.N // 2:
                    out[b] = val
                else:
                    out[self.N - b] = val.conjugate()
                    
        return np.array(out)

# ==========================================
# Automated Scaling Test Bench
# ==========================================
if __name__ == "__main__":
    target_sizes = [16, 32, 64, 128, 256, 512, 1024, 4096]
    
    print(f"{'N':<6} | {'Status':<15} | {'Max Error':<15}")
    print("-" * 45)
    
    for N in target_sizes:
        np.random.seed(42)
        x_test = np.random.rand(N).tolist()
        
        hardware_model = HardwareRFFT_Iterative(N)
        X_custom = hardware_model.compute(x_test)
        X_numpy = np.fft.rfft(x_test)
        
        try:
            np.testing.assert_allclose(X_custom, X_numpy, atol=1e-9)
            max_err = np.max(np.abs(X_custom - X_numpy))
            print(f"{N:<6} | SUCCESS         | {max_err:.4e}")
        except AssertionError as e:
            print(f"{N:<6} | FAILED          | CHECK LOGS")
            print(e)
            break