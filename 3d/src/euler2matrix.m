% EULER2MATRIX translate euler angle to rotation matrix
function Rs = euler2matrix( my_alpha, my_beta, my_gamma )
    num_frame = size(my_alpha, 1);
    Rs = zeros( num_frame, 9 );
    
    ca = cos(my_alpha); 
    sa = sin(my_alpha);
    cb = cos(my_beta);
    sb = sin(my_beta);
    cc = cos(my_gamma);
    sc = sin(my_gamma);
    for n = 1:num_frame
        
        C = [cc(n), sc(n),     0;
            -sc(n), cc(n),     0;
             0    ,     0,     1];
         
        B = [    1,     0,     0;
                 0, cb(n), sb(n);
                 0,-sb(n), cb(n)];
             
        A = [ca(n), sa(n),     0;
            -sa(n), ca(n),     0;
             0    ,     0,     1];

        R = C*B*A;

        Rs(n, :) = reshape(R', 1, 9);
    end
end