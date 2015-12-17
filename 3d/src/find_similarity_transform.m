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
function T = find_similarity_transform(x1, x2)
       
    num_scene     = size(x1,2);
    
    dof = 4;
    A = zeros(2*num_scene, dof);
    b = zeros(2*num_scene, 1);    
        
    for n = 1:num_scene
        A(2*n-1, :) = [x1(1,n) -x1(2,n) 1 0];
        A(2*n,   :) = [x1(2,n)  x1(1,n) 0 1];
        b(2*n-1, :) =  x2(1,n);
        b(2*n,   :) =  x2(2,n);
    end
    
    x = A\b;
    T = [x(1) -x(2) x(3)
         x(2)  x(1) x(4)
         0     0    1];
    
end

