import math
import numpy as np

# =====================================================================
# [VERILOG PARAMETERS] - State Machine Encoding
# =====================================================================
# FSM States
S_IDLE       = 0
S_FETCH      = 1
S_EXEC_REAL  = 2
S_EXEC_EQ7   = 3
S_EXEC_CFFT  = 4
S_SCHEDULE_1 = 5
S_SCHEDULE_2 = 6
S_NEXT_STAGE = 7
S_DONE       = 8

# Block Type Encodings (For FIFO Descriptors)
T_REAL       = 0
T_EQ7        = 1
T_CFFT       = 2
T_DONE_REAL  = 3
T_DONE_CFFT  = 4

# =====================================================================
# [VERILOG MODULE] - Hardware FIFO (Fixed Depth, Pointer Based)
# =====================================================================
class HW_FIFO:
    def __init__(self, depth):
        # Fixed memory allocation. No dynamic arrays allowed.
        # Data bus width accommodates: (Type, r_start, i_start, length, phi_step, base, binstep)
        self.memory = [(0, 0, 0, 0, 0, 0, 0)] * depth
        self.wr_ptr = 0
        self.rd_ptr = 0
        self.count = 0
        self.depth = depth

    def write(self, data):
        """Simulates asserting write_enable (we) for 1 clock cycle"""
        if self.count < self.depth:
            self.memory[self.wr_ptr] = data
            self.wr_ptr = (self.wr_ptr + 1) % self.depth
            self.count += 1
        else:
            raise OverflowError("Hardware FIFO Overflow! Increase depth.")

    def read(self):
        """Simulates asserting read_enable (re) for 1 clock cycle"""
        if self.count > 0:
            data = self.memory[self.rd_ptr]
            self.rd_ptr = (self.rd_ptr + 1) % self.depth
            self.count -= 1
            return data
        else:
            raise IndexError("Hardware FIFO Underflow! Tried to read empty FIFO.")

# =====================================================================
# [VERILOG TOP MODULE] - Cycle-Accurate RFFT
# =====================================================================
class HardwareRFFT_CycleAccurate:
    def __init__(self, N):
        if (N & (N - 1)) != 0 or N < 2:
            raise ValueError("N must be a power of 2")
        self.N = N
        self.total_stages = int(math.log2(N))
        
        # [MEMORY INSTANTIATION]
        # Main RAM (In-Place Execution)
        self.RAM = [0.0] * N
        
        # Dual Ping-Pong FIFOs (Depth of N is guaranteed to prevent overflow)
        self.fifo_A = HW_FIFO(N)
        self.fifo_B = HW_FIFO(N)

    def dsp_rotator(self, real_in, imag_in, phi):
        """
        [COMBINATIONAL LOGIC] 
        Simulates the DSP slice multiplier block and Twiddle ROM lookup.
        Executes purely combinationally within the EXEC states.
        """
        angle = -2.0 * math.pi * phi / self.N
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        
        r_out = real_in * cos_val - imag_in * sin_val
        i_out = imag_in * cos_val + real_in * sin_val
        return r_out, i_out

    def compute(self, x_in):
        # ---------------------------------------------------------
        # INITIALIZATION (Loading RAM and resetting FSM)
        # ---------------------------------------------------------
        for i in range(self.N):
            self.RAM[i] = float(x_in[i])

        # Write initial descriptor into FIFO A
        self.fifo_A.write((T_REAL, 0, 0, self.N, 1, 0, 1))

        # ---------------------------------------------------------
        # FSM REGISTERS (Declared outside the clock loop)
        # ---------------------------------------------------------
        state = S_FETCH
        stage_counter = 0
        active_fifo = 0 # 0 = Read A/Write B, 1 = Read B/Write A
        
        # Datapath / AGU Registers
        reg_type = 0
        reg_r_start = 0
        reg_i_start = 0
        reg_len = 0
        reg_phi_step = 0
        reg_base = 0
        reg_binstep = 0
        
        cycles_in_block = 0
        k = 0 # Butterfly counter
        
        # Performance tracker
        total_clock_cycles = 0

        # =========================================================
        # THE MASTER CLOCK (Every iteration = 1 Clock Cycle)
        # =========================================================
        while state != S_DONE:
            total_clock_cycles += 1
            
            # [MULTIPLEXERS] Route FIFOs based on active_fifo flag
            fifo_in  = self.fifo_A if active_fifo == 0 else self.fifo_B
            fifo_out = self.fifo_B if active_fifo == 0 else self.fifo_A

            # -----------------------------------------------------
            # FSM STATE LOGIC (Case Statement in Verilog)
            # -----------------------------------------------------
            
            if state == S_FETCH:
                if fifo_in.count > 0:
                    # Pop descriptor into FSM registers
                    desc = fifo_in.read()
                    reg_type, reg_r_start, reg_i_start, reg_len, reg_phi_step, reg_base, reg_binstep = desc
                    
                    cycles_in_block = reg_len // 2
                    k = 0 # Reset butterfly counter for new block
                    
                    # Pass-through logic for DONE descriptors
                    if reg_type == T_DONE_REAL or reg_type == T_DONE_CFFT:
                        fifo_out.write(desc)
                        state = S_FETCH
                    # Route to correct Datapath state
                    elif reg_type == T_REAL: state = S_EXEC_REAL
                    elif reg_type == T_EQ7:  state = S_EXEC_EQ7
                    elif reg_type == T_CFFT: state = S_EXEC_CFFT
                else:
                    # FIFO empty -> Current stage is complete
                    state = S_NEXT_STAGE

            # -----------------------------------------------------
            # DATAPATH EXECUTION STATES
            # -----------------------------------------------------
            elif state == S_EXEC_REAL:
                a_idx = reg_r_start + k
                b_idx = reg_r_start + k + cycles_in_block
                
                A = self.RAM[a_idx]
                B = self.RAM[b_idx]
                self.RAM[a_idx] = A + B
                self.RAM[b_idx] = A - B
                
                if k == cycles_in_block - 1:
                    state = S_SCHEDULE_1
                else:
                    k += 1

            elif state == S_EXEC_EQ7:
                r_idx = reg_r_start + k
                i_idx = reg_r_start + k + cycles_in_block
                
                A = self.RAM[r_idx]
                B = -self.RAM[i_idx] # Eq.7 Negation
                
                R, I = self.dsp_rotator(A, B, k * reg_phi_step)
                self.RAM[r_idx] = R
                self.RAM[i_idx] = I
                
                if k == cycles_in_block - 1:
                    state = S_SCHEDULE_1
                else:
                    k += 1

            elif state == S_EXEC_CFFT:
                r_a = reg_r_start + k
                r_b = reg_r_start + k + cycles_in_block
                i_a = reg_i_start + k
                i_b = reg_i_start + k + cycles_in_block
                
                A_r, B_r = self.RAM[r_a], self.RAM[r_b]
                A_i, B_i = self.RAM[i_a], self.RAM[i_b]
                
                R_top, I_top = A_r + B_r, A_i + B_i
                R_bot, I_bot = A_r - B_r, A_i - B_i
                
                R_rot, I_rot = self.dsp_rotator(R_bot, I_bot, k * reg_phi_step)
                
                self.RAM[r_a], self.RAM[i_a] = R_top, I_top
                self.RAM[r_b], self.RAM[i_b] = R_rot, I_rot
                
                if k == cycles_in_block - 1:
                    state = S_SCHEDULE_1
                else:
                    k += 1

            # -----------------------------------------------------
            # SCHEDULING STATES (Writing new descriptors)
            # -----------------------------------------------------
            elif state == S_SCHEDULE_1:
                if reg_type == T_REAL:
                    if cycles_in_block >= 2:
                        fifo_out.write((T_REAL, reg_r_start, 0, cycles_in_block, reg_phi_step * 2, reg_base, reg_binstep * 2))
                        state = S_SCHEDULE_2
                    else:
                        fifo_out.write((T_DONE_REAL, reg_r_start, 0, 2, 0, reg_base, reg_binstep))
                        state = S_FETCH
                        
                elif reg_type == T_EQ7:
                    if cycles_in_block >= 2:
                        fifo_out.write((T_CFFT, reg_r_start, reg_r_start + cycles_in_block, cycles_in_block, reg_phi_step * 4, reg_base, reg_binstep * 2))
                    else:
                        fifo_out.write((T_DONE_CFFT, reg_r_start, reg_r_start + 1, 2, 0, reg_base, reg_binstep))
                    state = S_FETCH # EQ7 only ever spawns 1 child block
                    
                elif reg_type == T_CFFT:
                    if cycles_in_block >= 2:
                        fifo_out.write((T_CFFT, reg_r_start, reg_i_start, cycles_in_block, reg_phi_step * 2, reg_base, reg_binstep * 2))
                    else:
                        fifo_out.write((T_DONE_CFFT, reg_r_start, reg_i_start, 2, 0, reg_base, reg_binstep))
                    state = S_SCHEDULE_2

            elif state == S_SCHEDULE_2:
                if reg_type == T_REAL:
                    fifo_out.write((T_EQ7, reg_r_start + cycles_in_block, 0, cycles_in_block, reg_phi_step, reg_base + reg_binstep, reg_binstep * 2))
                elif reg_type == T_CFFT:
                    if cycles_in_block >= 2:
                        fifo_out.write((T_CFFT, reg_r_start + cycles_in_block, reg_i_start + cycles_in_block, cycles_in_block, reg_phi_step * 2, reg_base + reg_binstep, reg_binstep * 2))
                    else:
                        fifo_out.write((T_DONE_CFFT, reg_r_start + 1, reg_i_start + 1, 2, 0, reg_base + reg_binstep, reg_binstep))
                state = S_FETCH

            # -----------------------------------------------------
            # STAGE TRANSITION (Ping-Pong Toggle)
            # -----------------------------------------------------
            elif state == S_NEXT_STAGE:
                stage_counter += 1
                active_fifo = 1 - active_fifo # Toggle Mux (0->1 or 1->0)
                
                if stage_counter == self.total_stages:
                    state = S_DONE
                else:
                    state = S_FETCH

        # =========================================================
        # [TESTBENCH LOGIC] - Output Unscrambling
        # Hardware leaves final values in RAM. We parse the final 
        # FIFO to extract them for validation.
        # =========================================================
        final_fifo = self.fifo_A if active_fifo == 0 else self.fifo_B
        out = [0j] * (self.N // 2 + 1)
        
        while final_fifo.count > 0:
            reg_type, r_start, i_start, _, _, base, binstep = final_fifo.read()
            
            if reg_type == T_DONE_REAL:
                bins = [(base, complex(self.RAM[r_start], 0)), 
                        (base + binstep, complex(self.RAM[r_start + 1], 0))]
            elif reg_type == T_DONE_CFFT:
                bins = [(base, complex(self.RAM[r_start], self.RAM[i_start]))]
            else:
                continue
                
            for b, val in bins:
                if b <= self.N // 2: out[b] = val
                else: out[self.N - b] = val.conjugate()
                    
        return np.array(out), total_clock_cycles

# ==========================================
# Automated Scaling Test Bench
# ==========================================
if __name__ == "__main__":
    target_sizes = [16, 32, 64, 128, 256, 512, 1024, 4096]
    
    print(f"{'N':<6} | {'Status':<10} | {'Cycles':<8} | {'Max Error':<15}")
    print("-" * 50)
    
    for N in target_sizes:
        np.random.seed(42)
        x_test = np.random.rand(N).tolist()
        
        hardware_model = HardwareRFFT_CycleAccurate(N)
        X_custom, clocks = hardware_model.compute(x_test)
        X_numpy = np.fft.rfft(x_test)
        
        try:
            np.testing.assert_allclose(X_custom, X_numpy, atol=1e-9)
            max_err = np.max(np.abs(X_custom - X_numpy))
            print(f"{N:<6} | SUCCESS    | {clocks:<8} | {max_err:.4e}")
        except AssertionError as e:
            print(f"{N:<6} | FAILED     | CHECK LOGS")
            print(e)
            break