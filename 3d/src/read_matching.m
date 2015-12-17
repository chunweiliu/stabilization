% READ_MATCHING
% input:
%  input_matching_file, c x 1 char array
% output:
%  x1, 2 x n left ransac feature matching points
%  x2, 2 x n right ransac feature matching points 
% a matching pair is [x y x y].
function [x1, x2] = read_matching(input_matching_file)
    
    fid = fopen(input_matching_file, 'r');
    
    
    % read all information from file
    num_matching = fscanf(fid, '%d', 1);
           
    x1 = zeros(2, num_matching);
    x2 = zeros(2, num_matching);
    
    for n = 1:num_matching
        r = fscanf( fid, '%g', [1 4] );
        x1(:, n) = [r(1); r(2)];
        x2(:, n) = [r(3); r(4)];
    end
    fclose(fid);
      
end