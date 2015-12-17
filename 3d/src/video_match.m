% VIDEO_MATCH match sift feature with ransac fundmaintal matrix filter
% input:
%  video_dir1, c x 1, left video dir
%  video_dir2, c x 1, right video dir
%  output_sift_dir, output_sift file dir
function video_match(key_dir1, key_dir2, ...
                     output_match_dir)
    
    RANSAC_T = 0.02;

    % Check if frame numbers are equal
    dats = dir([key_dir1 '*.key']);
    if length(dats) ~= length(dir([key_dir2 '*.key']))
        error('Number of key is different between %s and %s',...
               key_dir2, key_dir1);
    end
    
    % Check if output directory exist
    if (~exist(output_match_dir, 'dir'))
        cmd = ['mkdir ' output_match_dir];
        disp(cmd); 
        eval(cmd);
    end
    
    % For each frame, match SIFT feature if the matching is not exist
    fprintf(1, 'video_match: 0.00');
    num_frame = length(dats);
    for n = 1:num_frame
        fprintf(1, '\b\b\b\b%.2f', n/num_frame);
        
        output_file = sprintf('%s%04d.matching', output_match_dir, n-1);
        if exist(output_file, 'file'), continue; end
        
        key_file1 = sprintf('%s%04d.key', key_dir1, n-1);
        key_file2 = sprintf('%s%04d.key', key_dir2, n-1);
        
        [des1, loc1] = read_key(key_file1);
        [des2, loc2] = read_key(key_file2);
        
        % Start by david lowe's matching
        % For efficiency in Matlab, it is cheaper to compute dot products between
        %  unit vectors rather than Euclidean distances.  Note that the ratio of 
        %  angles (acos of dot products of unit vectors) is a close approximation
        %  to the ratio of Euclidean distances for small angles.
        %
        % distRatio: Only keep matches in which the ratio of vector angles from the
        %   nearest to second nearest neighbor is less than distRatio.
        distRatio = 0.6;   

        % For each descriptor in the first image, select its match to second image.
        des2t = des2';                          % Precompute matrix transpose
        for i = 1 : size(des1,1)
           dotprods = des1(i,:) * des2t;        % Computes vector of dot products
           [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results

           % Check if nearest neighbor has angle less than distRatio times 2nd.
           if (vals(1) < distRatio * vals(2))
              match(i) = indx(1);
           else
              match(i) = 0;
           end
        end

        % Return matching [x1 y1 x2 y2]
        num_matching = sum(match>0);        
        idx = 0;
        x1 = zeros(2, num_matching);        
        x2 = zeros(2, num_matching);        
        for i = 1: size(des1,1)
            if (match(i) > 0)
                idx = idx + 1;    
                x1(:, idx) = [loc1(i,2); loc1(i,1)];
                x2(:, idx) = [loc2(match(i),2); loc2(match(i),1)];
            end
        end
        
        % Filter the output by ransac
        [F, inliers] = ransac_fit_fundamental_matrix(x1, x2, RANSAC_T);
        x1 = x1(:, inliers);
        x2 = x2(:, inliers);
        % Write matching file out
        fid = fopen(output_file, 'w');
        fprintf(fid, '%d\n', size(x1,2));
        for m = 1:size(x1, 2)
            fprintf(fid, '%f %f %f %f\n', x1(1,m), x1(2,m), x2(1,m), x2(2,m));
        end
        fclose(fid);
        
    end
    fprintf(1, '\n');
end
