% CONTENT_PRESERVING_WARPS Siggraph 2010 paper implementation
% input:
%   im, n x m x 3, color image
%   matching_file, 
%   output_image_file
%   GRID_ROW_NUM
%   GRID_COL_NUM
%   SMOOTHNESS_ALPHA
function out = content_preserving_warps(im, matching_file, ...
    output_image_file, ...
    GRID_ROW_NUM, GRID_COL_NUM, SMOOTHNESS_ALPHA)
    
    if nargin < 4
        GRID_ROW_NUM     = 15;
        GRID_COL_NUM     = 20;
        SMOOTHNESS_ALPHA = 20;
    end
    
end