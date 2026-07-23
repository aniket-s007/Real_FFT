import math
import cmath
import numpy as np

# =====================================================================
# [MATH MODULE] - Combinational Multiplier
# =====================================================================
def rotator(real_in, imag_in, phi, N):
    """Combinational Multiplier Block (Stateless)"""
    angle = -2.0 * math.pi * phi / N
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    r_out = real_in * cos_val - imag_in * sin_val
    i_out = imag_in * cos_val + real_in * sin_val
    return r_out, i_out

# =====================================================================
# [DATAPATH MODULE] - Executes 1 cycle of a stage
# =====================================================================
def tick_stage(schedule_k, ram_in_read, ram_out_write, N):
    """
    Executes 1 cycle of processing for a dedicated stage.
    Reads from ram_in_read, writes to ram_out_write.
    """
    op, addr_A, addr_B, i_addr_A, i_addr_B, phi = schedule_k

    if op == 'IDLE':
        return

    # --- DATAPATH EXECUTION ---
    if op == 'REAL':
        A = ram_in_read[addr_A]
        B = ram_in_read[addr_B]
        ram_out_write[addr_A] = A + B
        ram_out_write[addr_B] = A - B

    elif op == 'EQ7':
        A = ram_in_read[addr_A]
        B = -ram_in_read[addr_B] # Eq.7 Negation
        R, I = rotator(A, B, phi, N)
        ram_out_write[addr_A] = R
        ram_out_write[addr_B] = I

    elif op == 'CFFT':
        A_r = ram_in_read[addr_A]
        B_r = ram_in_read[addr_B]
        A_i = ram_in_read[i_addr_A]
        B_i = ram_in_read[i_addr_B]
        
        Top_r, Top_i = A_r + B_r, A_i + B_i
        Bot_r, Bot_i = A_r - B_r, A_i - B_i
        
        R_rot, I_rot = rotator(Bot_r, Bot_i, phi, N)
        
        ram_out_write[addr_A] = Top_r
        ram_out_write[i_addr_A] = Top_i
        ram_out_write[addr_B] = R_rot
        ram_out_write[i_addr_B] = I_rot

# =====================================================================
# [CONTROL MODULE] - Address Generation Unit (AGU) Schedule Builder
# =====================================================================
def build_schedules(N):
    """Generates the cycle-by-cycle AGU control schedule for each stage."""
    num_stages = int(math.log2(N))
    stages_blocks = []
    current_blocks = [('REAL', 0, 0, N, 1, 0, 1)]
    
    for s in range(num_stages):
        stages_blocks.append(current_blocks)
        next_blocks = []
        for blk in current_blocks:
            b_type, r_start, i_start, length, phi_step, base, binstep = blk
            half = length // 2
            if b_type == 'REAL':
                if half >= 2:
                    next_blocks.append(('REAL', r_start, 0, half, phi_step*2, base, binstep*2))
                    next_blocks.append(('EQ7', r_start+half, 0, half, phi_step, base+binstep, binstep*2))
                else:
                    next_blocks.append(('DONE_REAL', r_start, 0, 2, 0, base, binstep))
            elif b_type == 'EQ7':
                if half >= 2:
                    next_blocks.append(('CFFT', r_start, r_start+half, half, phi_step*4, base, binstep*2))
                else:
                    next_blocks.append(('DONE_CFFT', r_start, r_start+1, 2, 0, base, binstep))
            elif b_type == 'CFFT':
                if half >= 2:
                    next_blocks.append(('CFFT', r_start, i_start, half, phi_step*2, base, binstep*2))
                    next_blocks.append(('CFFT', r_start+half, i_start+half, half, phi_step*2, base+binstep, binstep*2))
                else:
                    next_blocks.append(('DONE_CFFT', r_start, i_start, 2, 0, base, binstep))
                    next_blocks.append(('DONE_CFFT', r_start+1, i_start+1, 2, 0, base+binstep, binstep))
        current_blocks = next_blocks
        
    final_blocks = current_blocks
    
    # Flatten structures into explicit cycle schedules (N/2 clocks per stage)
    schedules = []
    for s in range(num_stages):
        sched = []
        for blk in stages_blocks[s]:
            b_type, r_start, i_start, length, phi_step, base, binstep = blk
            cycles = length // 2
            for k_blk in range(cycles):
                addr_A = r_start + k_blk
                addr_B = r_start + k_blk + cycles
                i_addr_A = i_start + k_blk
                i_addr_B = i_start + k_blk + cycles
                phi = k_blk * phi_step
                sched.append((b_type, addr_A, addr_B, i_addr_A, i_addr_B, phi))
        
        while len(sched) < N // 2:
            sched.append(('IDLE', 0, 0, 0, 0, 0))
        schedules.append(sched)
        
    return schedules, final_blocks

# =====================================================================
# [SYSTEM CONTROLLER] - Executes stage-by-stage pipeline on a frame
# =====================================================================
def process_frame(frame, N, schedules):
    """Passes data sequentially through the independent pipeline stages"""
    num_stages = int(math.log2(N))
    
    # Pre-allocate read and write buffers for each stage (acts as Ping-Pong RAM arrays)
    rams_read = [[0.0] * N for _ in range(num_stages + 1)]
    rams_write = [[0.0] * N for _ in range(num_stages + 1)]
    
    # 1. Load input into the first RAM
    for i in range(N):
        rams_read[0][i] = frame[i]
        
    # 2. Execute Dataflow
    for s in range(num_stages):
        for k in range(N // 2):
            # Pass the precise clock schedule and the physical memory slices to the tick function
            tick_stage(schedules[s][k], rams_read[s], rams_write[s+1], N)
        
        # Swap buffers: The output of this stage becomes the input of the next stage
        rams_read[s+1] = rams_write[s+1]
        rams_write[s+1] = [0.0] * N
            
    # Return the raw array from the final RAM block
    return rams_read[-1]

# =====================================================================
# [POST-PROCESSING] - Resolves the hardware scrambling
# =====================================================================
def unscramble_output(raw_ram, final_blocks, N):
    """Hardware unscrambler applied to the final output buffer"""
    out = [0j] * (N // 2 + 1)
    for blk in final_blocks:
        b_type, r_start, i_start, length, phi_step, base, binstep = blk
        
        bins_to_map = []
        if b_type == 'DONE_REAL':
            bins_to_map.append((base, complex(raw_ram[r_start], 0)))
            bins_to_map.append((base + binstep, complex(raw_ram[r_start + 1], 0)))
        elif b_type == 'DONE_CFFT':
            bins_to_map.append((base, complex(raw_ram[r_start], raw_ram[i_start])))
            
        for b, val in bins_to_map:
            if b <= N // 2:
                out[b] = val
            else:
                out[N - b] = val.conjugate()
                
    return np.array(out)


# =====================================================================
# Rigorous Sweep and Verification Testbench
# =====================================================================
if __name__ == "__main__":
    
    # Target scale table points
    target_sizes = [16, 32, 64, 128, 256, 512, 1024, 4096, 4096*2, 4096*16]
    
    print("\n==========================================================================")
    print("      FUNCTIONAL RFFT GLOBAL VERIFICATION RUN (NO CLASSES)                ")
    print("==========================================================================")
    print(f"{'FFT Size (N)':<14} | {'Verification Status':<22} | {'Max Absolute Error':<18}")
    print("-" * 62)
    
    for N in target_sizes:
        # Generate random inputs for unique frames
        np.random.seed(N + 42)
        x_original = np.random.rand(N).tolist()

        # 1. Compile the Hardware Schedules
        schedules, final_blocks = build_schedules(N)

        # 2. Execute hardware pipeline datapath functionally
        raw_ram_output = process_frame(x_original, N, schedules)
        
        # 3. Unscramble to final bins
        X_custom = unscramble_output(raw_ram_output, final_blocks, N)

        # 4. Compute reference forward transform
        X_numpy = np.fft.rfft(x_original)

        # 5. Assert precision thresholds
        try:
            np.testing.assert_allclose(X_custom, X_numpy, atol=1e-9)
            max_err = np.max(np.abs(X_custom - X_numpy))
            print(f"N = {N:<10} | PASS                   | {max_err:.4e}")
        except AssertionError as e:
            print(f"N = {N:<10} | FAIL                   | ERROR DETECTED")
            print(e)
            break
            
    print("-" * 62)
    
    # ==========================================================
    # RAW HARDWARE RAM MAPPING (Demonstration for N=16)
    # ==========================================================
    print("\n=====================================================================")
    print("      RAW RAM OUTPUT MAPPING DEMONSTRATION (For N=16)              ")
    print("=====================================================================")
    
    N_demo = 16
    x_demo = np.random.rand(N_demo).tolist()
    
    # Pure Functional Calls
    schedules_demo, blocks_demo = build_schedules(N_demo)
    raw_out_demo = process_frame(x_demo, N_demo, schedules_demo)

    # Build a map to show what each RAM index represents
    ram_map = ["(Unused/Idle)"] * N_demo
    for blk in blocks_demo:
        b_type, r_start, i_start, _, _, base, binstep = blk
        if b_type == 'DONE_REAL':
            ram_map[r_start] = f"Re(X_{base})"
            ram_map[r_start + 1] = f"Re(X_{base + binstep})"
        elif b_type == 'DONE_CFFT':
            ram_map[r_start] = f"Re(X_{base})"
            ram_map[i_start] = f"Im(X_{base})"
            
    print("RAM Idx |  Raw Value  | Represents")
    print("-" * 45)
    for i in range(N_demo):
        print(f"  {i:2d}    |  {raw_out_demo[i]:9.5f}  | {ram_map[i]}")