%IMWARP warp image via translation T.
% input:
%   im, n x m x 3, color image.
%   T, 3 x 3, homography.
function out = imwarp(im, T)
    xform = T';
    tform = maketform('projective', xform);    
    out = imtransform(im, tform, 'bicubic',...
                      'XData', [1 size(im,2)],...
                      'YData', [1 size(im,1)]);


end

