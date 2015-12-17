% READ_CAMERA
% input:
%  input_cam_file, c x 1 char array
% output:
%  Ks, n x 9 intrinsic of camera
%  Rs, n x 9 camera defined by world coordinates
%  ts, n x 3 translation defined by world coordinates
%  all above parameters saved as row per frame.
function [Ks, Rs, ts] = read_cameras( input_cam_file )
    
    fid = fopen( input_cam_file );
    % parse first 4 rows of comment
    comment = fgetl(fid);
    comment = fgetl(fid);
    comment = fgetl(fid);
    comment = fgetl(fid);
    
    % read all information from file
    num_frame = fscanf( fid, '%d', 1);
           
    Ks = zeros(num_frame, 9);
    Rs = zeros(num_frame, 9);
    ts = zeros(num_frame, 3);
    
    for n = 1:num_frame
        Ks(n,:) = fscanf( fid, '%g', [1 9] );
        Rs(n,:) = fscanf( fid, '%g', [1 9] );
        ts(n,:) = fscanf( fid, '%g', [1 3] );
    end
    fclose(fid);
      
end