function [x_derivative ] = DiscreteDifferentiate(x,delta_t)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
x_derivative = zeros(size(x)); % prealloc for speed

for n = 1:length(x)
    if(n == 1)
        x_derivative(n) = ( x(n + 1)- x(n) ) / delta_t;
    %elseif (n == length(x))
       % x_derivative(n) = ( x(n) - x(n - 1) ) / delta_t;
    else
        x_derivative(n) = ( x(n) - x(n - 1)) / delta_t;
    end
end

end

