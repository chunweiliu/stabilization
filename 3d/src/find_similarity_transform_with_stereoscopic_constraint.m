% FIND_SIMILARITY_TRANSFORM_WITH_STEREOSCOPIC_CONSTRAINT
% input:
%  x1_l, 2 x n shaky 2d left scene point.
%  x2_l, 2 x n smooth 2d left scene point.
%  x1_r, 2 x n shaky 2d right scene point.
%  x2_r, 2 x n smooth 2d right scene point.
%  m_l,  2 x m matching left features.
%  m_r,  2 x m matching right features.
%
%  x1_l, x2_l, x1_r, x2_r are found by reprojective 3d cameras.
%
% output:
%  T_l, 3 x 3 similarity transform matrix (4 dof).
%  T_r, 3 x 3 similarity transform matrix (4 dof).
function [T_l T_r] = find_similarity_transform_with_stereoscopic_constraint...
                     (x1_l, x2_l, x1_r, x2_r, m_l, m_r, W_STEREO)
       
    num_scene_l     = size(x1_l,2);
    num_scene_r     = size(x1_r,2);
    num_matching    = size(m_l, 2);

    dof = 8;
    A_l = zeros(2*num_scene_l, dof);
    b_l = zeros(2*num_scene_l, 1);
    A_r = zeros(2*num_scene_r, dof);
    b_r = zeros(2*num_scene_r, 1);
    A_m = zeros(2*num_matching, dof);
    b_m = zeros(2*num_matching, 1);
    
    oooo = [0 0 0 0];
    for n = 1:num_scene_l
        A_l(2*n-1, :) = [x1_l(1,n) -x1_l(2,n) 1 0 oooo];
        A_l(2*n,   :) = [x1_l(2,n)  x1_l(1,n) 0 1 oooo];
        b_l(2*n-1, :) =  x2_l(1,n);
        b_l(2*n,   :) =  x2_l(2,n);
    end
    for n = 1:num_scene_r
        A_r(2*n-1, :) = [oooo x1_r(1,n) -x1_r(2,n) 1 0];
        A_r(2*n,   :) = [oooo x1_r(2,n)  x1_r(1,n) 0 1];
        b_r(2*n-1, :) =  x2_r(1,n);
        b_r(2*n,   :) =  x2_r(2,n);
    end
    for n = 1:num_matching
        A_m(2*n-1, :) = [m_l(1,n) -m_l(2,n) 1 0 -m_r(1,n)  m_r(2,n) -1  0];
        A_m(2*n,   :) = [m_l(2,n)  m_l(1,n) 0 1 -m_r(2,n) -m_r(1,n)  0 -1];
        b_m(2*n-1, :) =  m_l(1,n) - m_r(1,n);
        %b_m(2*n,   :) =  0;
    end
    
    A = [A_l; A_r; W_STEREO*A_m];
    b = [b_l; b_r; W_STEREO*b_m];
    x = A\b;
    T_l = [x(1) -x(2) x(3)
           x(2)  x(1) x(4)
           0     0    1];
    T_r = [x(5) -x(6) x(7)
           x(6)  x(5) x(8)
           0     0    1];
end

