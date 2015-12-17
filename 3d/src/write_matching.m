% WRITE_MATCHING
% input:
%  x1, 2 x n left ransac feature matching points
%  x2, 2 x n right ransac feature matching points 
%  output_matching_file, c x 1 char array

% a matching pair is [x y x y].
function write_matching(x1, x2, output_matching_file)
    
    if size(x1,2) ~= size(x2,2)
        error('x1 and x2 are not equal size');
    end
    
    
    num_matching = size(x1,2);
    
    fid = fopen(output_matching_file, 'w');
    fprintf(fid, '%d\n', num_matching);
          
    for n = 1:num_matching
        fprintf( fid, '%f %f %f %f\n', ...
        x1(1, n), x1(2, n), x2(1, n), x2(2, n));
    end
    fclose(fid);
      
end

