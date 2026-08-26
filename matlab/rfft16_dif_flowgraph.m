function rfft16_dif_flowgraph()
% RFFT16_DIF_FLOWGRAPH  16-point real-input FFT, coded to LOOK like the
% flow graph (Fig. 3, Garrido/Parhi/Grajal, "A Pipelined FFT Architecture
% for Real-Valued Signals," IEEE TCAS-I, 2009).
%
% Each STAGE below is one vertical slice of that picture: a "top" block
% of rows that stays real, a "bottom" block that gets Eq.(7)'d into a
% complex signal, and -- wherever the figure draws a boxed number -- a
% twiddle() rotation by that box's phi. Read the code stage by stage with
% the figure next to it and the two should line up 1:1.
%
% Run this file directly (F5, or type its name) to compare against
% MATLAB's built-in fft().

N = 16;

x = randn(1, N);            % real input, x(1) = paper's x_0, ..., x(16) = x_15
% x = 1:N;                  % <- swap in a fixed, hand-traceable input instead

[X, S] = rfft16_flowgraph(x, N);          % our flow graph, bins X_0..X_8

Xref      = fft(x, N);                    % MATLAB's own answer
Xref_half = Xref(1:N/2+1);                % keep only the non-redundant half

fprintf('  k |        our flow graph X_k          |            MATLAB fft(x)            |  abs err\n');
fprintf('----|-------------------------------------|--------------------------------------|----------\n');
for k = 0:N/2
    fprintf('%3d | %9.5f %+9.5fj          | %9.5f %+9.5fj           | %.2e\n', ...
        k, real(X(k+1)), imag(X(k+1)), ...
        real(Xref_half(k+1)), imag(Xref_half(k+1)), ...
        abs(X(k+1) - Xref_half(k+1)));
end
fprintf('\nmax abs error vs fft(x): %.3e\n', max(abs(X - Xref_half)));

% Uncomment to see the raw, still-scrambled RAM contents after each stage
% (useful for tracing a single wire through the picture):
% disp(S)

end


function [X, S] = rfft16_flowgraph(x, N)
% RFFT16_FLOWGRAPH  Hand-mapped Fig. 3 flow graph. x is a real 1x16 row.
% Returns X = [X_0 .. X_8] (the non-redundant rfft bins) and S = the raw
% contents of the RAM after each of the 4 stages (still in hardware/
% scrambled order, exactly as drawn on the wires in the figure).

assert(isreal(x) && isequal(size(x), [1 N]));

%% ---------------- STAGE 1 : plain real butterfly, rows (i, i+8) --------
% No twiddle box anywhere in Stage 1 of the figure -> no rotation here.
top = x(1:8);
bot = x(9:16);
s1 = zeros(1, N);
s1(1:8)  = top + bot;    % rows 0-7  -> feeds another real-FFT half
s1(9:16) = top - bot;    % rows 8-15 -> feeds the Eq.(7) box next stage

%% ---------------- STAGE 2 ------------------------------------------------
s2 = zeros(1, N);

% rows 0-7 (still real): plain butterfly again, still no rotation box
top = s1(1:4);
bot = s1(5:8);
s2(1:4) = top + bot;
s2(5:8) = top - bot;

% rows 8-15: Eq.(7) box -- negate the bottom half, then rotate by the
% four boxed numbers "0 1 2 3" drawn in Fig.3 Stage 2
A = s1(9:12);
B = -s1(13:16);                     % Eq.(7) negation (the "-1" edges)
[r, im] = twiddle(A, B, 0:3, N);    % boxes: 0  1  2  3
s2(9:12)  = r;
s2(13:16) = im;

%% ---------------- STAGE 3 -------------------------------------------------
s3 = zeros(1, N);

% rows 0-3 (real): plain butterfly, no rotation
top = s2(1:2);
bot = s2(3:4);
s3(1:2) = top + bot;
s3(3:4) = top - bot;

% rows 4-7 (real): Eq.(7) box, boxed numbers "0 2"
A = s2(5:6);
B = -s2(7:8);
[r, im] = twiddle(A, B, [0 2], N);
s3(5:6) = r;
s3(7:8) = im;

% rows 8-15 (already complex: re = s2(9:12), im = s2(13:16)): a radix-2
% complex-FFT butterfly -- the circled "Real .. comes together" block in
% the figure -- with boxed numbers "0 4" (phi=4 is a trivial j-rotation,
% which is exactly why the figure notes it "same as real": no multiplier
% needed, just a swap-and-negate)
Ar = s2(9:10);   Br = s2(11:12);
Ai = s2(13:14);  Bi = s2(15:16);
s3(9:10)  = Ar + Br;                       % top half, real part
s3(13:14) = Ai + Bi;                       % top half, imag part
[Rr, Ri]  = twiddle(Ar - Br, Ai - Bi, [0 4], N);
s3(11:12) = Rr;                            % bottom half, real part
s3(15:16) = Ri;                            % bottom half, imag part

%% ---------------- STAGE 4 (final) -------------------------------------------
s4 = zeros(1, N);

% row 0-1 (real): plain butterfly -> final bins X_0 (DC), X_8 (Nyquist)
s4(1) = s3(1) + s3(2);
s4(2) = s3(1) - s3(2);

% row 2-3 (real): Eq.(7) box, phi = 0 -> feeds bin X_4
[r, im] = twiddle(s3(3), -s3(4), 0, N);
s4(3) = r;
s4(4) = im;

% rows 4-7 (complex): CFFT butterfly, phi = 0 -> bins X_2 (direct), X_6 (folded)
Ar = s3(5); Br = s3(6); Ai = s3(7); Bi = s3(8);
s4(5) = Ar + Br;  s4(7) = Ai + Bi;
[Rr, Ri] = twiddle(Ar - Br, Ai - Bi, 0, N);
s4(6) = Rr;  s4(8) = Ri;

% rows 8-11 (complex): CFFT butterfly, phi = 0 -> bins X_1 (direct), X_7 (folded)
Ar = s3(9); Br = s3(10); Ai = s3(13); Bi = s3(14);
s4(9)  = Ar + Br;  s4(13) = Ai + Bi;
[Rr, Ri] = twiddle(Ar - Br, Ai - Bi, 0, N);
s4(10) = Rr;  s4(14) = Ri;

% rows 12-15 (complex): CFFT butterfly, phi = 0 -> bins X_5 (direct), X_3 (folded)
Ar = s3(11); Br = s3(12); Ai = s3(15); Bi = s3(16);
s4(11) = Ar + Br;  s4(15) = Ai + Bi;
[Rr, Ri] = twiddle(Ar - Br, Ai - Bi, 0, N);
s4(12) = Rr;  s4(16) = Ri;

%% ---------------- OUTPUT UNSCRAMBLE ------------------------------------------
% The hardware writes bins out of order, and some land at a "virtual" bin
% number above N/2 that must be conjugate-folded back onto the real
% non-redundant range: X_(N-b) = conj(raw value written at virtual bin b).
%
% columns: [ real-part RAM index , imag-part RAM index (0 = purely real) , virtual bin ]
raw_map = [ 1   0   0
            2   0   8
            3   4   4
            5   7   2
            6   8  10
            9  13   1
           10  14   9
           11  15   5
           12  16  13 ];

X = zeros(1, N/2+1);
for row = 1:size(raw_map, 1)
    re_idx = raw_map(row, 1);
    im_idx = raw_map(row, 2);
    vbin   = raw_map(row, 3);
    if im_idx == 0
        val = complex(s4(re_idx), 0);
    else
        val = complex(s4(re_idx), s4(im_idx));
    end
    if vbin <= N/2
        X(vbin + 1) = val;
    else
        X(N - vbin + 1) = conj(val);
    end
end

S = struct('s1', s1, 's2', s2, 's3', s3, 's4', s4);

end


function [re_out, im_out] = twiddle(re_in, im_in, phi, N)
% TWIDDLE  The boxed rotator in the figure: multiply (re_in + j*im_in) by
% exp(-j*2*pi*phi/N). phi may be a vector (applies elementwise, one
% column of the RAM at a time -- exactly like the parallel wires in the
% picture).
ang = -2*pi*phi/N;
re_out = re_in .* cos(ang) - im_in .* sin(ang);
im_out = im_in .* cos(ang) + re_in .* sin(ang);
end
