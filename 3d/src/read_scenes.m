% read_scene read the acts' output scene file.
% input:
%  name, c x 1, acts' output scnes file
% output:
%  Xs, 3 x n, 3D scenes points.
function Xs = read_scenes(name)    

    %threshold = 1e5;
    
    fid = fopen( name );
    % parse first row
    comment = fgetl(fid);
    
    num_scenes = fscanf( fid, '%d', 1 );
    
    Xs = zeros(num_scenes,3);
    num_inlier = 0;        
    for n = 1:num_scenes
        x = fscanf( fid, '%g', [1 3] );
        %if norm(x) > threshold, continue; end
        
        num_inlier = num_inlier + 1;
        Xs(num_inlier, :) = x;
    end
    fclose(fid);
    
    Xs = Xs(1:num_inlier, :);
    Xs = Xs';
    
end