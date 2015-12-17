% VIDEO_FIND_STEREO_SIMILARITY
% 2.5D method find a similarity transform prewarping
% input:
%  vidname
%  vidframe
%  WEIGHT_STEREO
%  output_path_l_file, c x 1, warping for left frame
%  output_path_r_file, c x 1, warping for right frame
function video_find_stereo_similarity(vidname, vidframe, WEIGHT_STEREO, ...
         output_matching_l_dir, output_matching_r_dir, ...
         output_path_l_file, output_path_r_file, ...
         output_video_l_dir, output_video_r_dir)
     
    % Check if need to draw figures
    if nargin < 6
        flag_draw = 0;
    else
        flag_draw = 1;
        % Check if output directory exist
        if (~exist(output_video_l_dir, 'dir'))
            cmd = ['mkdir ' output_video_l_dir];
            disp(cmd); 
            eval(cmd);
        end
        if (~exist(output_video_r_dir, 'dir'))
            cmd = ['mkdir ' output_video_r_dir];
            disp(cmd); 
            eval(cmd);
        end
    end
    % Check if output directory exist
    if (~exist(output_matching_l_dir, 'dir'))
        cmd = ['mkdir ' output_matching_l_dir];
        disp(cmd); 
        eval(cmd);
    end
    if (~exist(output_matching_r_dir, 'dir'))
        cmd = ['mkdir ' output_matching_r_dir];
        disp(cmd); 
        eval(cmd);
    end
    
    % Check if the file exist 
    fprintf(1, 'video_find_stereo_similarity: 0.00');
    %if exist(output_path_l_file, 'file') && exist(output_path_r_file, 'file')
    %    error('\b\b\b\b1.00');
    %end            
    
    points_l_dir     = sprintf('../log/points/%s_l/', vidname);
    points_r_dir     = sprintf('../log/points/%s_r/', vidname);
    points_l_smo_dir = sprintf('../log/points/%s_l_smooth/', vidname);
    points_r_smo_dir = sprintf('../log/points/%s_r_smooth/', vidname);
    matching_dir     = sprintf('../log/matching/%s/', vidname);
    for n = 1:vidframe        
        fprintf(1, '\b\b\b\b%.2f', n/vidframe);
        na   = sprintf('%s%04d.points', points_l_dir, n-1);
        x1_l = read_points(na);
        
        na   = sprintf('%s%04d.points', points_l_smo_dir, n-1);
        x2_l = read_points(na);
        
        na   = sprintf('%s%04d.points', points_r_dir, n-1);
        x1_r = read_points(na);
        
        na   = sprintf('%s%04d.points', points_r_smo_dir, n-1);
        x2_r = read_points(na);
        
        na   = sprintf('%s%04d.matching', matching_dir, n-1);
        [m_l, m_r] = read_matching(na);
        
        [T_l T_r] = find_similarity_transform_with_stereoscopic_constraint...
                    (x1_l, x2_l, x1_r, x2_r, m_l, m_r, WEIGHT_STEREO);
                
        % Write smooth matching of frame
        % you have to write x2 from another camera
        outl = sprintf('%s%04d.matching', output_matching_l_dir, n-1);        
        %write_matching(x1_l, x2_l, outl);
        
        x1_l_with_constraint = T_l * [x1_l; ones(1, size(x1_l,2))];
        T_s = find_similarity_transform(x2_l, x1_l_with_constraint);
        x2_l_with_constraint = T_s * [x2_l; ones(1, size(x2_l,2))];
        write_matching(x1_l, x2_l_with_constraint, outl);
        
        outr = sprintf('%s%04d.matching', output_matching_r_dir, n-1);
        %write_matching(x1_r, x2_r, outr);
        
        %x2_r_with_constraint = T_r * [x1_r; ones(1, size(x1_r,2))];
        x1_r_with_constraint = T_r * [x1_r; ones(1, size(x1_r,2))];        
        T_s = find_similarity_transform(x2_r, x1_r_with_constraint);
        x2_r_with_constraint = T_s * [x2_r; ones(1, size(x2_r,2))];
        write_matching(x1_r, x2_r_with_constraint, outr);
        
        % debug
        %{
        im = zeros(480,616);
        figure(1)
        plot_features(im, x1_l_with_constraint, '.r');
        name = sprintf('../tmp/x1_l_with_constraint_project_%04d.png',n-1);
        print(name, '-dpng');
        figure(2)
        plot_features(im, x2_l_with_constraint, '.g');
        name = sprintf('../tmp/x2_l_with_constraint_project_%04d.png',n-1);
        print(name, '-dpng');
        figure(3)
        plot_features(im, x2_l, '.b');
        name = sprintf('../tmp/x2_l_project_%04d.png', n-1);
        print(name, '-dpng');
        %}
        
        % Write warp as similarity transform style
        write_path(T_l, output_path_l_file);
        write_path(T_r, output_path_r_file);
        
        % write output image
        if flag_draw == 1
            outl = sprintf('%s%04d.jpg', output_video_l_dir, n-1);
            outr = sprintf('%s%04d.jpg', output_video_r_dir, n-1);
            if exist(outl, 'file') && exist(outr, 'file')
                continue
            end
            
            na = sprintf('../dat/img/%s_l/%04d.jpg', vidname, n-1);
            im = imread(na);
            im_warp = imwarp(im, T_l);                        
            imwrite(im_warp, outl);

            na = sprintf('../dat/img/%s_r/%04d.jpg', vidname, n-1);
            im = imread(na);
            im_warp = imwarp(im, T_r);            
            imwrite(im_warp, outr);
        end
        
    end
    
    fprintf(1, '\n');
end

function write_path(T, na)
    
    %scale = (T(1,1)^2 + T(1,2)^2)^0.5;
    %theta = acos(T(1,1));
    %tx    = T(1,3);
    %ty    = T(2,3);    
    %fprintf(fid, '%f %f %f %f\n', scale, theta, tx, ty);
    fid = fopen(na, 'a');
    a  = T(1,1);
    b  = T(2,1);
    tx = T(1,3);
    ty = T(2,3);
    fprintf(fid, '%f %f %f %f\n', a, b, tx, ty);
    fclose(fid);
end