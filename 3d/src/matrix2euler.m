% MATRIX2EULER translate rotation matrix to euler angle
function [my_alpha, my_beta, my_gamma] = matrix2euler( Rs )
    
    num_frame = size(Rs, 1);
    my_alpha = zeros( num_frame, 1 ); % around z
    my_beta  = zeros( num_frame, 1 ); % around x
    my_gamma = zeros( num_frame, 1 ); % around z'
        
    for n = 1:num_frame
        R = reshape(Rs(n, :), 3, 3)';
                        
        % here we get radious
        my_alpha(n) = atan2 (R(3,1), -R(3,2)); 
        my_beta(n)  = atan2 ((R(3,1)^2 + R(3,2)^2)^0.5, R(3,3) );
        my_gamma(n) = atan2 (R(1,3), R(2,3));                      
    end
    
end