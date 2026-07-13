import math
import cmath

def rotator(real_in, imag_in, phi, N=16):
    """
    Hardware-equivalent complex rotator block (the numbered boxes in Fig 3).
    Calculates (real_in + j*imag_in) * e^(-j * 2*pi*phi / N)
    """
    angle = -2.0 * math.pi * phi / N
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    
    real_out = real_in * cos_val - imag_in * sin_val
    imag_out = imag_in * cos_val + real_in * sin_val
    
    return real_out, imag_out

def rfft_16point_dif(x):
    """
    16-point Decimation-In-Frequency RFFT based on Garrido et al. Fig 3.
    """
    if len(x) != 16:
        raise ValueError("Input must be exactly 16 real samples.")

    # ---------------------------------------------------------
    # STAGE 1: Purely real distance-8 butterflies
    # ---------------------------------------------------------
    x1 = [0.0] * 16
    for i in range(8):
        x1[i]   = x[i] + x[i+8]
        x1[i+8] = x[i] - x[i+8]

    # ---------------------------------------------------------
    # STAGE 2: Real butterflies and the Eq. 7 Complex Split
    # ---------------------------------------------------------
    x2 = [0.0] * 16
    for i in range(4):
        x2[i]   = x1[i] + x1[i+4]
        x2[i+4] = x1[i] - x1[i+4]

    for i in range(4):
        r_out, i_out = rotator(real_in=x1[i+8], imag_in=-x1[i+12], phi=i, N=16)
        x2[i+8]  = r_out
        x2[i+12] = i_out

    # ---------------------------------------------------------
    # STAGE 3: Distance-2 butterflies and Rotators
    # ---------------------------------------------------------
    x3 = [0.0] * 16
    
    # Upper Half (Indices 0..7)
    for i in range(2):
        x3[i]   = x2[i] + x2[i+2]
        x3[i+2] = x2[i] - x2[i+2]

    x3[4], x3[6] = rotator(x2[4], -x2[6], phi=0, N=16)
    x3[5], x3[7] = rotator(x2[5], -x2[7], phi=2, N=16)

    # Lower Half (Indices 8..15)
    # Step 1: Distance-2 butterflies on the separated Real and Imag rails
    u_real = [0.0] * 4
    u_imag = [0.0] * 4
    for i in range(2):
        u_real[i]   = x2[8+i] + x2[10+i]
        u_real[i+2] = x2[8+i] - x2[10+i]
        
        u_imag[i]   = x2[12+i] + x2[14+i]
        u_imag[i+2] = x2[12+i] - x2[14+i]

    # Step 2: Complex rotators crossing the rails (This was the missing bug fix)
    phis = [0, 0, 0, 4]
    # for i in range(4):
    #     r, img = rotator(u_real[i], u_imag[i], phi=phis[i], N=16)
    #     x3[8+i]  = r
    #     x3[12+i] = img

    for i in range(2):
        r1, img1 = rotator(u_real[i], u_imag[i], phi=phis[i], N=16)
        x3[8+i]  = r1
        x3[10+i] = img1
        r2, img2 = rotator(u_real[i+2], u_imag[i+2], phi=phis[i+2], N=16)
        x3[12+i] = r2
        x3[14+i] = img2


    # ---------------------------------------------------------
    # STAGE 4: Distance-1 butterflies and final Rotators
    # ---------------------------------------------------------
    x4 = [0.0] * 16
    x4[0] = x3[0] + x3[1]
    x4[1] = x3[0] - x3[1]

    x4[2], x4[3] = rotator(x3[2], -x3[3], phi=0, N=16)

    for base in [4, 6, 8, 10, 12, 14]:
        x4[base]   = x3[base] + x3[base+1]
        x4[base+1] = x3[base] - x3[base+1]

    # ---------------------------------------------------------
    # OUTPUT MAPPING: Unscrambling the hardware outputs
    # ---------------------------------------------------------
    X = [0j] * 16 
    
    X[0] = complex(x4[0], 0)
    X[8] = complex(x4[1], 0)

    X[4] = complex(x4[2], x4[3])   # R4, I4
    X[12] = complex(x4[2], -x4[3]) # conj(R4, I4)

    X[2] = complex(x4[4], x4[6])   # R2, I2
    X[14] = complex(x4[4], -x4[6]) # conj(R2, I2)

    X[10] = complex(x4[5], x4[7])  # R10, I10
    X[6] = complex(x4[5], -x4[7])  # conj(R10, I10)

    X[1] = complex(x4[8], x4[10])  # R1, I1
    X[15] = complex(x4[8], -x4[10]) # conj(R1, I1)

    X[9] = complex(x4[9], x4[11])  # R9, I9
    X[7] = complex(x4[9], -x4[11]) # conj(R9, I9)

    X[5] = complex(x4[12], x4[14]) # R5, I5
    X[11] = complex(x4[12], -x4[14])# conj(R5, I5)

    X[13] = complex(x4[13], x4[15]) # R13, I13
    X[3] = complex(x4[13], -x4[15])# conj(R13, I13)

    # # Conjugate symmetries (BUG FIX: The original code had a bug here, which is now corrected)
    # X[6] = complex(x4[5], -x4[7])  # conj(R10, I10)
    # X[7] = complex(x4[9], -x4[11]) # conj(R9, I9)
    # X[3] = complex(x4[13], -x4[15])# conj(R13, I13)

    return X

# ==========================================
# Test Bench / Validation
# ==========================================

# if __name__ == "__main__":
#     import numpy as np

#     np.random.seed(42)
#     x_test = np.random.rand(16).tolist()

#     X_custom = rfft_16point_dif(x_test)
#     X_numpy = np.fft.fft(x_test)[:9]

#     print("Bin | Custom Model (Fig 3)           | Numpy FFT")
#     print("-" * 65)
#     for k in range(9):
#         print(f"{k:2d}  | {X_custom[k].real:8.4f} + {X_custom[k].imag:8.4f}j | {X_numpy[k].real:8.4f} + {X_numpy[k].imag:8.4f}j")
#         assert cmath.isclose(X_custom[k], X_numpy[k], abs_tol=1e-9), f"Mismatch at bin {k}"
        
#     print("-" * 65)
#     print("SUCCESS: Golden model perfectly matches standard FFT!")

if __name__ == "__main__":
    import numpy as np

    np.random.seed(42)
    x_test = np.random.rand(16).tolist()

    # 1. Run both models (np.fft.rfft automatically handles the 9-bin output)
    X_custom = rfft_16point_dif(x_test)
    X_numpy = np.fft.fft(x_test)

    # 2. Verify all elements instantly without a loop
    np.testing.assert_allclose(X_custom, X_numpy, atol=1e-9, err_msg="FFT Mismatch!")

    # 3. Print success (and optionally print the rounded arrays if you want to inspect them)
    print("SUCCESS: Golden model perfectly matches standard FFT!")
    print(np.round(X_custom, 4))