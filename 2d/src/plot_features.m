%PLOT_STATISTIC plot vertical and horizontal disparity offset.

function plot_features( input_filename, output_filename )
    
    path_deshaker_dir= '../log/txt/path_deshaker/';
    path_along_dir   = '../log/txt/path_along/';
    path_joint_dir   = '../log/txt/path_joint/';
    
    %{
    warp_along_dir   = '../log/png/warp_along/';
    warp_joint_dir   = '../log/png/warp_joint/';
    warp_deshaker_dir= '../log/png/warp_deshaker/';
        
    warp_along_dir1  = [warp_along_dir input_filename '_s+u_1e1_l/'];
    warp_along_dir2  = [warp_along_dir input_filename '_s+u_1e1_r/'];
    warp_joint_dir1  = [warp_joint_dir input_filename '_s+u_1e0_d+a_1e-1_l/'];
    warp_joint_dir2  = [warp_joint_dir input_filename '_s+u_1e0_d+a_1e-1_r/'];
    warp_deshaker_dir1 = [warp_deshaker_dir input_filename '_deshaker_l/'];
    warp_deshaker_dir2 = [warp_deshaker_dir input_filename '_deshaker_r/'];
        
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
    
    %im = imread([im_dir1 imgs1(1).name]);
    %im = imread('csiegirl_4in1_0163.png');
    
    
    
    
    
    %[im_r, im_c] = size(im);
    
    sift_dir_path    = '../log/txt/sift_fund_matching/';
    sift_dir1  = [sift_dir_path input_filename '_l/'];
    sift_dir2  = [sift_dir_path input_filename '_r/'];
    dats1 = dir( [sift_dir1 '*.key'] );
    dats2 = dir( [sift_dir2 '*.key'] );
    if length( dats1 ) ~= length( dats2 )
        error('The left and right keyfiles number are not consistency');
    end
    %num_frame = length(dats1); % has cut as user study
    num_frame = 211; % children
    %num_frame = 211; % csiegirl
    %num_frame = 261; % redgirl
    
    % read deshaker log
    path_deshaker_file1 = [path_deshaker_dir input_filename '_deshaker_l.path' ];
    path_deshaker_file2 = [path_deshaker_dir input_filename '_deshaker_r.path' ];
    path_deshaker_l = read_path(path_deshaker_file1, num_frame);
    path_deshaker_r = read_path(path_deshaker_file2, num_frame);
    
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
    
    fprintf(1, 'Process: 0.00');
    %for n = 1:num_frame
    for n = 163
        fprintf(1, '\b\b\b\b%.2f', n/num_frame);
        [loc1, des1] = read_key( [sift_dir1 dats1(n).name] );
        [loc2, des2] = read_key( [sift_dir2 dats2(n).name] );
        
        x1 = [loc1(2,:); loc1(1,:); ones(1, size(loc1, 2))];
        x2 = [loc2(2,:); loc2(1,:); ones(1, size(loc2, 2))];
        
        
        figure(1)
        im = imread('csiegirl_0163.png');
        imshow(im)
        hold on        
        feature_idx = find_line_features(x1, x2);
        hold off
                        
        figure(2)
        im = imread('csiegirl_deshaker_0163.png');
        imshow(im)
        path1 = path_deshaker_l(n,:);
        path2 = path_deshaker_r(n,:);
        [d_d, wxd1, wxd2] = calculate_d( path1, x1, path2, x2 );                
        line_features(wxd1, wxd2, feature_idx);
        hold off
        
        figure(3)
        im = imread('csiegirl_iccv09_0163.png');
        imshow(im)
        hold on
        path1 = path_iccv09_l(n,:);
        path2 = path_iccv09_r(n,:);
        [d_i, wxi1, wxi2] = calculate_d( path1, x1, path2, x2 );        
        line_features(wxi1, wxi2, feature_idx);
        hold off
        
        figure(4)
        im = imread('csiegirl_purposed_0163.png');
        imshow(im)
        hold on
        path1 = path_purposed_l(n,:);
        path2 = path_purposed_r(n,:);
        [d_p, wxp1, wxp2] = calculate_d( path1, x1, path2, x2 );                        
        line_features(wxp1, wxp2, feature_idx);
        hold off
        
    end
    fprintf(1, '\n');
    
end

function [out, wx1, wx2] = calculate_d( path1, x1, path2, x2 ) % x1 = [x, y, 1]
    a   = path1(1);
    b   = path1(2);
    tx  = path1(3);
    ty  = path1(4);
    M_l = [ a -b tx+30;
            b  a ty-20;
            0  0  1];
        
    a   = path2(1);
    b   = path2(2);
    tx  = path2(3);
    ty  = path2(4);
    M_r = [ a -b tx+30;
            b  a ty-20;
            0  0  1];
            
    wx1 = M_l*x1;
    wx2 = M_r*x2;
    %out = abs(wx1 - wx2);
    out = wx1 - wx2;
end

function feature_idx = find_line_features( x1, x2 )
    
    num_feature = size(x1, 2);       
    feature_idx = [];
    for n = 1 : num_feature
        if( norm(x1(:,n)-x2(:,n)) > 15 && x1(2,n) < 460)
            
            line([x1(1, n), x2(1, n)], ...
                 [x1(2, n), x2(2, n)], ...
                 'Marker', 'o', ...
                 'linewidth', 2)            
            feature_idx = [feature_idx n];
        end
    end   
    
end

function line_features( x1, x2, feature_idx )
            
    for n = feature_idx
        if( norm(x1(:,n)-x2(:,n)) > 15 )
            
            line([x1(1, n), x2(1, n)], ...
                 [x1(2, n), x2(2, n)], ...
                 'Marker', 'o', ...
                 'linewidth', 2)
            
        end
    end    
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
