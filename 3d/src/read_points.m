% READ_CAMERA
% input:
%  input_matching_file, c x 1 char array
% output:
%  xs, 2 x n left ransac feature matching points

% a point is [x y].
function xs = read_points(input_points_file)
    
    fid = fopen(input_points_file, 'r');
    
    
    % read all information from file
    num_points = fscanf(fid, '%d', 1);
           
    xs = zeros(2, num_points);    
    
    for n = 1:num_points
        r = fscanf( fid, '%g', [1 2] );
        xs(:, n) = [r(1); r(2)];        
    end
    fclose(fid);
      
end