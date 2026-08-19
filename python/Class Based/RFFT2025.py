import math
import cmath
import numpy as np

# =====================================================================
# [HARDWARE MODULE] - A Single Pipeline Stage (PE + Delay Line)
# =====================================================================
class PipelineStage:
    def __init__(self, stage_idx, delay_size, N):
        self.stage_idx = stage_idx
        self.delay_size = delay_size
        self.N = N
        
        # Hardware Shift Register (FIFO)
        self.shift_reg = [0j] * delay_size
        self.counter = 0

        # Twiddle ROM for this specific stage
        # In hardware, this is an address generator driving a LUT
        self.twiddle_ROM = []
        for i in range(delay_size):
            phi = i * (N // (2 * delay_size))
            angle = -2.0 * math.pi * phi / N
            self.twiddle_ROM.append(complex(math.cos(angle), math.sin(angle)))

    def tick(self, x_in, debug=False):
        """Executes 1 clock cycle for this specific PE"""
        
        # The FSM state alternates every 'delay_size' clock cycles
        state = (self.counter // self.delay_size) % 2
        rom_addr = self.counter % self.delay_size
        
        # Read the oldest value from the shift register
        delayed_val = self.shift_reg.pop(0)
        
        if state == 0:
            # STATE 0: Filling the delay line. 
            # Incoming data goes straight into the shift register.
            # The PE passes the delayed value to the next stage.
            self.shift_reg.append(x_in)
            out_val = delayed_val
            
            if debug and self.stage_idx == 0:
                print(f"  [Stage {self.stage_idx}] Mode: FILL | Storing input, passing delayed data.")
                
        else:
            # STATE 1: Butterfly & Rotate.
            # The shift register is full. We butterfly the incoming data 
            # with the delayed data.
            A = delayed_val
            B = x_in
            
            # Butterfly
            top_out = A + B
            bot_out = A - B
            
            # Rotate bottom output and feed it BACK into the shift register
            rotated_bot = bot_out * self.twiddle_ROM[rom_addr]
            self.shift_reg.append(rotated_bot)
            
            # The top output flows immediately to the next stage
            out_val = top_out
            
            if debug and self.stage_idx == 0:
                print(f"  [Stage {self.stage_idx}] Mode: CALC | Butterfly completed. Passing Top Out.")

        self.counter += 1
        return out_val


# =====================================================================
# [TOP MODULE] - The Serial Pipeline Architecture
# =====================================================================
class SerialPipelinedFFT:
    def __init__(self, N, debug=False):
        if (N & (N - 1)) != 0 or N < 2:
            raise ValueError("N must be a power of 2")
        self.N = N
        self.stages = int(math.log2(N))
        self.debug = debug
        
        # Instantiate the chain of PEs (Stage 0 has delay N/2, Stage 1 has N/4, etc.)
        self.pipeline = []
        for s in range(self.stages):
            delay = N // (2 ** (s + 1))
            self.pipeline.append(PipelineStage(s, delay, N))

    def tick(self, x_in):
        """Clocks the entire pipeline by 1 cycle"""
        current_val = x_in
        # Data flows sequentially through every PE in the pipeline
        for stage in self.pipeline:
            current_val = stage.tick(current_val, self.debug)
        return current_val

    def bit_reverse_index(self, i):
        """Hardware bit-reversal address generator"""
        b = f"{i:0{self.stages}b}"
        return int(b[::-1], 2)

    def process_stream(self, input_array):
        """Simulates feeding the data array cycle-by-cycle into the chip"""
        if len(input_array) != self.N:
            raise ValueError("Input length must match N")
            
        raw_outputs = []
        
        if self.debug:
            print("\n--- BEGIN HARDWARE CLOCKING ---")

        # 1. Feed the N inputs into the pipeline
        for clock, val in enumerate(input_array):
            if self.debug:
                print(f"\n[Clock {clock:03d}] Feeding Input[{clock}] = {val:.4f}")
            out = self.tick(complex(val, 0))
            raw_outputs.append(out)
            
        # 2. Pipeline Flush
        # A pipelined FFT has an inherent latency of N-1 cycles.
        # We must clock the chip N-1 more times with zeros to flush the 
        # remaining calculated data out of the delay lines.
        for clock in range(self.N, 2 * self.N - 1):
            if self.debug:
                print(f"\n[Clock {clock:03d}] Feeding Zeros (Flushing Pipeline)")
            out = self.tick(0j)
            raw_outputs.append(out)

        # The valid outputs start appearing after the pipeline fills (cycle N-1)
        valid_pipeline_outputs = raw_outputs[self.N - 1:]

        # 3. Unscramble the Output (Bit-Reversal)
        # Because we fed Natural Input, the hardware yields Bit-Reversed Output.
        unscrambled = [0j] * self.N
        for i in range(self.N):
            reversed_idx = self.bit_reverse_index(i)
            unscrambled[reversed_idx] = valid_pipeline_outputs[i]

        return np.array(unscrambled)


# =====================================================================
# Rigorous End-to-End Verification Testbench
# =====================================================================
if __name__ == "__main__":
    
    # 1. Configuration
    N = 32
    DEBUG_MODE = False # Set to True to watch the cycle-by-cycle streaming
    
    np.random.seed(99)
    # Generate random real-valued input stream
    x_stream = np.random.rand(N)

    # 2. Clock data through the Serial Pipeline Hardware Model
    hardware = SerialPipelinedFFT(N, debug=DEBUG_MODE)
    X_custom_full = hardware.process_stream(x_stream)
    
    # For a Real-FFT, we only care about the first (N/2)+1 unique bins
    X_custom_rfft = X_custom_full[:N//2 + 1]

    # 3. Standard Numpy RFFT
    X_numpy = np.fft.rfft(x_stream)

    # ==========================================================
    # FORWARD FFT COMPARISON
    # ==========================================================
    print("================================================================================")
    print(f"      STREAMING PIPELINE OUTPUT vs NUMPY (N={N} BINS)      ")
    print("================================================================================")
    print("Bin | Pipeline Hardware Output         | Numpy RFFT Output")
    print("-" * 80)
    for k in range(N // 2 + 1):
        c_val = X_custom_rfft[k]
        n_val = X_numpy[k]
        match = " " if cmath.isclose(c_val, n_val, abs_tol=1e-9) else "*"
        print(f" {k:2d}{match} | {c_val.real:10.5f} + {c_val.imag:10.5f}j | {n_val.real:10.5f} + {n_val.imag:10.5f}j")
    
    print("-" * 80)
    try:
        np.testing.assert_allclose(X_custom_rfft, X_numpy, atol=1e-9)
        print("[PASS] Forward FFT: All frequency bins match perfectly.")
    except AssertionError:
        print("[FAIL] Forward FFT mismatch detected.")

    # ==========================================================
    # INVERSE FFT RECOVERY (The Ultimate Verification)
    # ==========================================================
    x_recovered = np.fft.irfft(X_custom_rfft, n=N)

    print("\n=====================================================================")
    print(f"      TIME-DOMAIN SIGNAL RECOVERY (FIRST 10 SAMPLES)       ")
    print("=====================================================================")
    print("Idx | Streamed Input   | Recovered (IFFT) | Error Margin")
    print("-" * 69)
    for i in range(10):
        err = abs(x_stream[i] - x_recovered[i])
        print(f"{i:3d} |   {x_stream[i]:10.6f}     |   {x_recovered[i]:10.6f}       |  {err:.2e}")
    print("-" * 69)

    try:
        np.testing.assert_allclose(x_recovered, x_stream, atol=1e-9)
        print("[PASS] Inverse FFT: Signal perfectly recovered from pipeline output!")
    except AssertionError:
        print("[FAIL] Inverse FFT mismatch detected.")