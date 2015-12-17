% REPROJECT_3D_SCENES_TO_2D (for smoothing)
% input:
%  camera_file , acts' camera file.
%  scenes_3d_file, 3d scenes.
%  output_scenes_2d_file, 2d scenes.
function reproject_3d_scenes_to_2d(cameras_file, scenes_3d_file, ...                                   
                                   output_scenes_2d_dir, ...
                                   cameras_file2, output_scenes_2d_dir2,...
                                   FRAME_WIDTH, FRAME_HEIGHT)
    
    %FRAME_WIDTH  = 640;
    %FRAME_HEIGHT = 480;
    
    % Check if output directory exist
    if (~exist(output_scenes_2d_dir, 'dir'))
        cmd = ['mkdir ' output_scenes_2d_dir];
        disp(cmd); 
        eval(cmd);
    end
    if (~exist(output_scenes_2d_dir2, 'dir'))
        cmd = ['mkdir ' output_scenes_2d_dir2];
        disp(cmd); 
        eval(cmd);
    end

    [Ks, Rs, ts]    = read_cameras(cameras_file);
    [Ks2, Rs2, ts2] = read_cameras(cameras_file2);
    Xs              = read_scenes(scenes_3d_file);
    num_scenes      = size(Xs,2);    
    
    fprintf(1, 'reproject_3d_scenes_to_2d: 0.00');
    num_frame = size(Ks,1);
    for n = 1:num_frame
        fprintf(1, '\b\b\b\b%.2f', n/num_frame);
        outputfile  = sprintf('%s%04d.points', output_scenes_2d_dir, n-1);
        outputfile2 = sprintf('%s%04d.points', output_scenes_2d_dir2, n-1);
        if exist(outputfile, 'file') && exist(outputfile2, 'file')
            continue
        end
        
        % Read camera's world coordinates
        K  = reshape(Ks(n,:), 3, 3)';
        R  = reshape(Rs(n,:), 3, 3)';
        t  = ts(n,:)';
        % Transform it to camera coordinates
        t  = -(R\t);
        
        % Read smooth camera as above
        K2 = reshape(Ks2(n,:), 3, 3)';
        R2 = reshape(Rs2(n,:), 3, 3)';
        t2 = ts2(n,:)';
        t2 = -(R2\t2);
    
        
        % homogenous make operator linearly
        %Xs = [Xs; ones(1,num_scenes)];        
        %P  = [R t];
        
        % or for loop
        xs  = zeros(2, num_scenes); 
        xs2 = zeros(2, num_scenes); 
        idx = 0;
        for m = 1:num_scenes
            % Reproject 3d to 2d
            x = K * (R \ Xs(:,m) + t);
            x = [x(1)/x(3); x(2)/x(3)];
            
            % Leave the invisible points out
            % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if x(1) < 0 || x(2) < 0 ...
            || x(1) > FRAME_WIDTH || x(2) > FRAME_HEIGHT
                 continue;
            end
            idx = idx + 1;
            xs(:,idx) = x;
            
            % Reproject smooth points
            x = K2 * (R2 \ Xs(:,m) + t2);
            x = [x(1)/x(3); x(2)/x(3)];
            xs2(:,idx) = x;
        end
      
        % Write output file        
        fid = fopen(outputfile, 'w');
        fprintf(fid, '%d\n', idx);        
        for m = 1:idx
            fprintf(fid, '%f %f\n', xs(1,m), xs(2,m));
        end
        fclose(fid);
        
        % Write output smooth file        
        fid = fopen(outputfile2, 'w');
        fprintf(fid, '%d\n', idx);        
        for m = 1:idx
            fprintf(fid, '%f %f\n', xs2(1,m), xs2(2,m));
        end
        fclose(fid);
                
    end
    fprintf(1,'\n');
end