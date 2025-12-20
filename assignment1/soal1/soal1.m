clear all;
clf;

Fs = 100;             % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 50 * Fs;           % Length of signal
t = (0:L-1)*T;        % Time vector

y = 0.1 - 9 * cos(2*pi*t) + sin(12*pi*t).^2;

Y = fft(y);

plot(Fs/L*(0:L-1),abs(Y),"LineWidth",3)
title("Complex Magnitude of fft Spectrum")
xlabel("f (Hz)")
ylabel("|fft(X)|")