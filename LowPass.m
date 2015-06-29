%%
% This function applies a low pass filter to the data being analyzed.
% fc is the cutoff frequency
% M is the kernel size
function smoothed_data = LowPass(data, fc)
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
end