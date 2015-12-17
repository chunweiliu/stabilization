%FIND_TRANSFORM Summary of this function goes here
%   input:
%       input_key_dir1
%       input_key_dir2
%       output_path_file
%   output:

function find_transform( input_key_dir1, input_key_dir2, output_path_file )
    
    dats1 = dir( [input_key_dir1 '*.key']);
    dats2 = dir( [input_key_dir2 '*.key']);
    if length(dats1) ~= length(dats2)
        error( 'frame number inconsistent' );
    end
    
    num_frame = length(dats1);
    fprintf(1, 'Process: 0.00');
    fid = fopen(output_path_file, 'w');
    for n = 1:num_frame
        fprintf(1, '\b\b\b\b%.2f', n/num_frame);
        
        [loc1, des1] = read_key( [input_key_dir1 dats1(n).name] );
        [loc2, des2] = read_key( [input_key_dir2 dats2(n).name] );
        num_feature = size(loc1, 2);
        % use similarity transform gauss deshaker's warping
        b = [loc2(1,:)'; loc2(2,:)']; % b = [x1 x2 ... xn, y1 y2 ... yn]
        A = [loc1(1,:)', -loc1(2,:)', ones(num_feature, 1), zeros(num_feature, 1);
             loc1(2,:)',  loc1(1,:)', zeros(num_feature, 1), ones(num_feature, 1)];
        x = A\b;
        fprintf(fid, '%f %f %f %f\n', x(1), x(2), x(3), x(4));
    end
    fclose(fid);
    fprintf(1, '\n');
    

end

function [loc, des] = read_key( key_file_name )
    
    g = fopen( key_file_name, 'r');

    if g == -1
        error('Could not open file tmp.key.');
    end
    [header, count] = fscanf(g, '%d %d', [1 2]);
    if count ~= 2
        error('Invalid keypoint file beginning.');
    end
    num = header(1);
    len = header(2);
    if len ~= 128
        error('Keypoint descriptor length invalid (should be 128).');
    end

    % Creates the two output matrices (use known size for efficiency)
    locs = double(zeros(num, 4));
    descriptors = double(zeros(num, 128));

    % Parse tmp.key
    for i = 1:num
        [vector, count] = fscanf(g, '%f %f %f %f', [1 4]); %row col scale ori
        if count ~= 4
            error('Invalid keypoint file format');
        end
        locs(i, :) = vector(1, :);

        [descrip, count] = fscanf(g, '%d', [1 len]);
        if (count ~= 128)
            error('Invalid keypoint file value.');
        end
        % Normalize each input vector to unit length
        descrip = descrip / sqrt(sum(descrip.^2));
        descriptors(i, :) = descrip(1, :);
    end
    fclose(g);

    loc = locs'; % 4 x n
    des = descriptors'; % 128 x n
end