%PLOT_STATISTIC plot vertical and horizontal disparity offset.

function plot_statistic( input_filename, output_filename )
    
    path_deshaker_dir= '../log/txt/path_deshaker/';
    path_along_dir   = '../log/txt/path_along/';
    path_joint_dir   = '../log/txt/path_joint/';
    
    warp_along_dir   = '../log/png/warp_along/';
    warp_joint_dir   = '../log/png/warp_joint/';
    warp_deshaker_dir= '../log/png/warp_deshaker/';
    
    %warp_along_dir1  = [warp_along_dir input_filename '_s+u_1e1_l/'];
    %warp_along_dir2  = [warp_along_dir input_filename '_s+u_1e1_r/'];
    %warp_joint_dir1  = [warp_joint_dir input_filename '_s+u_1e0_d+a_1e-1_l/'];
    %warp_joint_dir2  = [warp_joint_dir input_filename '_s+u_1e0_d+a_1e-1_r/'];
    warp_deshaker_dir1 = [warp_deshaker_dir input_filename '_deshaker_l/'];
    warp_deshaker_dir2 = [warp_deshaker_dir input_filename '_deshaker_r/'];
    
    %{
    im_along1 = dir([warp_along_dir1 '*.png']);
    im_along2 = dir([warp_along_dir2 '*.png']);
    im_joint1 = dir([warp_joint_dir1 '*.png']);
    im_joint2 = dir([warp_joint_dir2 '*.png']);
    im_deshaker1 = dir([warp_deshaker_dir1 '*.png']);
    im_deshaker2 = dir([warp_deshaker_dir2 '*.png']);
    %}
    
    im_dir = '../dat/img/';
    im_dir1 = [im_dir input_filename '_l/'];
    imgs1 = dir([im_dir1 '*.png']);
    im_dir2 = [im_dir input_filename '_r/'];
    imgs2 = dir([im_dir2 '*.png']);
    
    im1 = imread([im_dir1 imgs1(1).name]);
    %im_r = size(im1, 1);
    %im_c = size(im1, 2);
    
    sift_dir_path    = '../log/txt/sift_fund_matching/';
    sift_dir1  = [sift_dir_path input_filename '_l/'];
    sift_dir2  = [sift_dir_path input_filename '_r/'];
    dats1 = dir( [sift_dir1 '*.key'] );
    dats2 = dir( [sift_dir2 '*.key'] );
    if length( dats1 ) ~= length( dats2 )
        error('The left and right keyfiles number are not consistency');
    end
    %num_frame = length(dats1); % has cut as user study
    %num_frame = 211; % children
    num_frame = 211; % csiegirl
    %num_frame = 261; % redgirl
    
    % read deshaker log
    %path_deshaker_file1 = [path_deshaker_dir input_filename '_deshaker_l.log' ];
    %path_deshaker_file2 = [path_deshaker_dir input_filename '_deshaker_r.log' ];
    %path_deshaker_l = read_log(path_deshaker_file1, num_frame);
    %path_deshaker_r = read_log(path_deshaker_file2, num_frame);
    path_deshaker_file1 = [path_deshaker_dir input_filename '_deshaker_l.path' ];
    path_deshaker_file2 = [path_deshaker_dir input_filename '_deshaker_r.path' ];
    path_deshaker_l = read_path(path_deshaker_file1, num_frame);
    path_deshaker_r = read_path(path_deshaker_file2, num_frame);
    
    im_dir = '../log/png/warp_deshaker/';
    im_deshaker_dir1 = [im_dir input_filename '_deshaker_l/'];
    im_deshaker_l = dir([im_deshaker_dir1 '*.png']);
    im_deshaker_dir2 = [im_dir input_filename '_deshaker_r/'];
    im_deshaker_r = dir([im_deshaker_dir2 '*.png']);
    
    % read iccv09 path
    path_iccv09_file1 = [path_along_dir input_filename '_s+u_1e1_l.path'];
    path_iccv09_file2 = [path_along_dir input_filename '_s+u_1e1_r.path'];
    path_iccv09_l = read_path(path_iccv09_file1, num_frame);
    path_iccv09_r = read_path(path_iccv09_file2, num_frame);
    
    % read purposed path
    path_purposed_file1 = [path_joint_dir input_filename '_s+u_1e0_d+a_1e-1_l.path'];
    path_purposed_file2 = [path_joint_dir input_filename '_s+u_1e0_d+a_1e-1_r.path'];
    path_purposed_l = read_path(path_purposed_file1, num_frame);
    path_purposed_r = read_path(path_purposed_file2, num_frame);
    
    %original_d  = zeros(num_frame, 3);
    beforeM1 = eye(3);
    beforeM2 = eye(3);
    deshaker_d  = zeros(num_frame, 3);
    iccv09_d    = zeros(num_frame, 3);
    purposed_d  = zeros(num_frame, 3);
    fprintf(1, 'Process: 0.00');
    for n = 1:num_frame
        fprintf(1, '\b\b\b\b%.2f', n/num_frame);
        [loc1, des1] = read_key( [sift_dir1 dats1(n).name] );
        [loc2, des2] = read_key( [sift_dir2 dats2(n).name] );
        
        x1 = [loc1(2,:); loc1(1,:); ones(1, size(loc1, 2))];
        x2 = [loc2(2,:); loc2(1,:); ones(1, size(loc2, 2))];
        
        % plot average original disparity in this frame
        %original_d(n, :) = mean(abs(x1-x2), 2);
        %original_d(n, :) = mean(x1-x2, 2);
        d_o = x1-x2;
        d_o(:,2) = 0; %pefect camera should not have vertical offset
        %{
        d_o = x1-x2;
        figure(1)
        im_name1 = [im_dir1 imgs1(n).name];
        im1 = imread( im_name1 );
        out_name = sprintf('../log/png/vis_features/%s_%04d_l.png', input_filename, n-1);
        plot_features( im1, x1, out_name );
        
        figure(2)
        im_name2 = [im_dir2 imgs2(n).name];
        im2 = imread( im_name2 );
        out_name = sprintf('../log/png/vis_features/%s_%04d_r.png', input_filename, n-1);
        plot_features( im2, x2, out_name );
        %}
        
        % plot average deshaker's disparity in this frame        
        %deshaker_d(n, :) = mean(calculate_d(path_deshaker_l(n,:), x1, ...
        %                                    path_deshaker_r(n,:), x2), 2);
        
        % aggregation of deshaker's motion
        %{
        if n == 1
            d_d = x1-x2;
            wx1 = x1;
            wx2 = x2;
        else
            [d_d, wx1, wx2, beforeM1, beforeM2] = ...
                calculate_deshaker_d(beforeM1, path_deshaker_l(n,:), x1, ...
                                     beforeM2, path_deshaker_r(n,:), x2, ...
                                     im_r, im_c);                                 
        end
        %}
        [d_d, wx1, wx2] = calculate_d(path_deshaker_l(n,:), x1, path_deshaker_r(n,:), x2);
        %deshaker_d(n, :) = sum((d_d-d_o).*(d_d-d_o), 2)./size(x1,2);
        deshaker_d(n, :) = sum(abs(d_d-d_o), 2)./size(x1,2);
        
        %{
        figure(1)
        im_name1 = [warp_deshaker_dir1 im_deshaker_l(n).name];
        im1 = imread( im_name1 );
        out_name = sprintf('../log/png/vis_features/%s_deshaker_%04d_l.png', input_filename, n-1);
        plot_features( im1, wx1, out_name );
        
        figure(2)
        im_name2 = [warp_deshaker_dir2 im_deshaker_r(n).name];
        im2 = imread( im_name2 );
        out_name = sprintf('../log/png/vis_features/%s_deshaker_%04d_r.png', input_filename, n-1);
        plot_features( im2, wx2, out_name );
        %}
        
        % plot average iccv09's disparity in this frame
        %iccv09_d(n, :) = mean(calculate_d(path_iccv09_l(n,:), x1, ...
        %                                  path_iccv09_r(n,:), x2), 2);
        d_i = calculate_d(path_iccv09_l(n,:), x1, path_iccv09_r(n,:), x2);
        %iccv09_d(n, :) = sum((d_i-d_o).*(d_i-d_o), 2)./size(x1,2);
        iccv09_d(n, :) = sum(abs(d_i-d_o), 2)./size(x1,2);
        %{
        [d_i, wx1, wx2] = calculate_d(path_iccv09_l(n,:), x1, path_iccv09_r(n,:), x2);
        iccv09_d(n, :) = mean(abs(d_i-d_o), 2);
        figure(1)
        im1 = imread([warp_along_dir1 im_along1(n).name]);
        out_name = sprintf('../log/png/vis_features/%s_iccv09_%04d_l.png', input_filename, n-1);
        plot_features(im1, wx1, out_name);
        figure(2)
        im2 = imread([warp_along_dir2 im_along2(n).name]);
        out_name = sprintf('../log/png/vis_features/%s_iccv09_%04d_r.png', input_filename, n-1);
        plot_features(im2, wx2, out_name);
        %}
        
        % plot average purposed's disparity in this frame
        %purposed_d(n, :) = mean(calculate_d(path_purposed_l(n,:), x1, ...
        %                                    path_purposed_r(n,:), x2), 2);
        d_p = calculate_d(path_purposed_l(n,:), x1, path_purposed_r(n,:), x2);
        %purposed_d(n, :) = sum( (d_p-d_o).*(d_p-d_o), 2 )./size(x1,2);
        purposed_d(n, :) = sum( abs(d_p-d_o), 2 )./size(x1,2);
        %{
        d_p = calculate_d(path_purposed_l(n,:), x1, path_purposed_r(n,:), x2);
        purposed_d(n, :) = mean(abs(d_p-d_o),2);
        figure(1)
        im1 = imread([warp_joint_dir1 im_joint1(n).name]);
        out_name = sprintf('../log/png/vis_features/%s_purposed_%04d_l.png', input_filename, n-1);
        plot_features(im1, wx1, out_name);
        figure(2)
        im2 = imread([warp_joint_dir2 im_joint2(n).name]);
        out_name = sprintf('../log/png/vis_features/%s_purposed_%04d_r.png', input_filename, n-1);
        plot_features(im2, wx2, out_name);
        %}
    end
    fprintf(1, '\n');
    %original_d  = original_d(:, 1:2);
    deshaker_d  = deshaker_d(:, 1:2);
    iccv09_d    = iccv09_d(:, 1:2);
    purposed_d  = purposed_d(:, 1:2);
    %save( output_filename, 'original_d', 'deshaker_d', 'iccv09_d', 'purposed_d' );        
    save( output_filename, 'deshaker_d', 'iccv09_d', 'purposed_d' );
    
    font_size = 24;
    line_width = 3;
    x = 1:num_frame;
    figure(1)    
    plot( x, deshaker_d(:,1), 'b', 'LineWidth', line_width ); 
    hold on
    plot( x, iccv09_d(:,1), 'r', 'LineWidth', line_width );
    plot( x, purposed_d(:,1), 'g', 'LineWidth', line_width);
    hold off        
    xlabel( 'Frame', 'FontSize', font_size );    
    ylabel( 'Disparity difference', 'FontSize', font_size );    
    h_legend = legend( 'Deshaker', 'ICCV09', 'Proposed',  2);
    set(h_legend,'FontSize',font_size);
    saveas( gcf, [output_filename(1:end-4) '_dx.fig' ] );    
    print([output_filename(1:end-4) '_dx.pdf' ], '-dpdf' );
    
    figure(2)    
    plot( x, deshaker_d(:,2), 'b', 'LineWidth', line_width ); 
    hold on
    plot( x, iccv09_d(:,2), 'r', 'LineWidth', line_width );
    plot( x, purposed_d(:,2), 'g', 'LineWidth', line_width);
    hold off        
    xlabel( 'Frame', 'FontSize', font_size );    
    ylabel( 'Disparity difference', 'FontSize', font_size );        
    h_legend = legend( 'Deshaker', 'ICCV09', 'Proposed', 2 );    
    set(h_legend,'FontSize',font_size);

    saveas( gcf, [output_filename(1:end-4) '_dy.fig' ] );
    
    print( [output_filename(1:end-4) '_dy.pdf'], '-dpdf' );
    
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

function out = read_path( filename, num_frame )
    out = zeros( num_frame, 4 );
    fid = fopen( filename, 'r' );
    for n = 1:num_frame
        [vector, count] = fscanf(fid, '%f %f %f %f', [1 4]); % [a b tx ty]
        if count ~= 4
            error( 'Invalid path file format' );
        end
        out(n, :) = vector;
    end
end

function [out, wx1, wx2] = calculate_d( path1, x1, path2, x2 ) % x1 = [x, y, 1]
    a   = path1(1);
    b   = path1(2);
    tx  = path1(3);
    ty  = path1(4);
    M_l = [ a -b tx;
            b  a ty;
            0  0  1];
        
    a   = path2(1);
    b   = path2(2);
    tx  = path2(3);
    ty  = path2(4);
    M_r = [ a -b tx;
            b  a ty;
            0  0  1];
            
    wx1 = M_l*x1;
    wx2 = M_r*x2;
    %out = abs(wx1 - wx2);
    out = wx1 - wx2;
end
%{
function out = read_log( filename, num_frame )
    out = zeros( num_frame, 4 );
    fid = fopen( filename, 'r' );
    for n = 2:num_frame
        [vector, count] = fscanf(fid, '%d %f %f %f %f', [1 5]); % [f x y t s]
        if count ~= 5
            error( 'Invalid log file format' );
        end
        %tx    = vector(2);
        %ty    = vector(3);
        %theta = vector(4)/180.; % degree -> radian
        %scale = vector(5);
        %a = scale*cos(theta);
        %b = scale*sin(theta);
        %out(n, :) = [a b tx ty];
        out(n,:) = vector(2:end);
    end
    %out(num_frame, :) = [1 0 0 0];
    out(1, :) = [1 0 0 0];
end
function [out, wx1, wx2, M1, M2] = ...
    calculate_deshaker_d( before_M1, path1, x1, ...
                          before_M2, path2, x2, ...
                          im_r, im_c )
    one_unit_deshaker_degree = -1/60;    
    K = [1 0 -im_c/2;
         0 1 -im_r/2;
         0 0 1];
     
    panx = path1(1);
    pany = path1(2);
    theta_d = path1(3);
    scale = path1(4);      
    theta = theta_d * one_unit_deshaker_degree;
    a = scale*cos(theta);
    b = scale*sin(theta);
    c = panx;
    d = pany;
    this_M = [a -b  c;
              b  a  d;
              0  0  1];
    M1 = before_M1 * inv(K) * this_M * K;
    
    panx = path2(1);
    pany = path2(2);
    theta_d = path2(3);
    scale = path2(4);      
    theta = theta_d * one_unit_deshaker_degree;
    a = scale*cos(theta);
    b = scale*sin(theta);
    c = panx;
    d = pany;
    this_M = [a -b  c;
              b  a  d;
              0  0  1];
    M2 = before_M2 * inv(K) * this_M * K;
        
    wx1 = M1\x1;
    wx2 = M2\x2;
    out = wx1 - wx2;
end

function [out, M]= deshaker_unit_converter( before_M, panx, pany, theta_d, scale, im_r, im_c )
    one_unit_deshaker_degree = -1/60;
    theta = theta_d * one_unit_deshaker_degree;
    a = scale*cos(theta);
    b = scale*sin(theta);
    c = panx;
    d = pany;    
    this_M = [a -b  c;
              b  a  d;
              0  0  1];
    M = beforeM * this_M;
    invM = inv(M);
    out = [invM(1,1), invM(2,1), invM(1,3), invM(2,3)];
    
end
%}
function plot_features( im, x, out_name )
    num_feature = size(x, 2);
    imshow(im)
    hold on
    for n = 1 : num_feature
        plot(x(1, n), x(2, n), '.r')
    end
    hold off
    saveas(gcf, out_name );
end