% SOLVE_ACCELERATION solve parameter by following term
% x = argmin \sum_{n=2}^N-1 |x(n-1) - 2x(n) + x(n+1)|^2 + 
%            \sum_{n=1}^N   |x(n) - p(n)|^2
% input:
%   p - observation
%   w - weight
% output:
%   x
function x = solve_acceleration( p, w_motion, w_smooth )

    num_frame = size(p, 1);
    
    b = zeros(2*num_frame - 2, 1);
    b (end-num_frame+1:end, 1) = w_smooth.*p;
    
    aa = zeros(num_frame-2, num_frame);
    for n = 1:num_frame-2
        aa(n, n:n+2) = [1 -2 1];
    end
    A = [w_motion.*aa;
         w_smooth.*diag(ones(num_frame,1))];
    x = A\b;
end