%%
% This function applies a low pass filter to the data being analyzed.

function Analyze_FFT(data)
Fs = 1000;                    % Sampling frequency
T = 1/Fs;                     % Sample time
L = length(data);             % Length of signal
t = (0:L-1)*T;                % Time vector

NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(data,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);
plot(f,2*abs(Y(1:NFFT/2+1)))
hold

fc = 0.006;
%fc = 0.05;
M = 100;
H = double.empty;
for I=0:M
    if((I - floor(M/2)) == 0)
        H(I+1) = 2*pi*fc;
    else
        H(I+1) = sin(2*pi*fc * (I-M/2))/(I-M/2);
    end
    H(I+1) = H(I+1) * (0.54 - 0.46 * cos(2*pi*I/M));
end

H = H/sum(H);

smoothed_data = conv(data,H,'same');

L = length(smoothed_data);             % Length of signal
t = (0:L-1)*T;                % Time vector

NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(smoothed_data,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);
plot(f,2*abs(Y(1:NFFT/2+1)),'r');

figure;
plot(t, data);
hold;
plot(t, smooth(data,21), 'r');
plot(t, smoothed_data, 'g');
end
