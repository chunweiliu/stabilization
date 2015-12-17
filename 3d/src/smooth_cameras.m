% SMOOTH_CAMERA
% input:
%  cam_file           , c x 1, ACTS' output camera format
%  output_smo_cam_file, c x 1
function smooth_cameras(cam_file, output_smo_cam_file)
    
    WEIGHT_DATA               = 100;
    WEIGHT_SMOOTH_TRANSLATION = 2;
    WEIGTH_SMOOTH_ROTATION    = 1;
    
    [Ks, Rs, ts] = read_cameras(cam_file);
    
    % Smooth Ks
    
    % Smooth Rs
    w_motion = WEIGHT_DATA;
    w_smooth = WEIGTH_SMOOTH_ROTATION;
    [my_alpha, my_beta, my_gamma] = matrix2euler(Rs);
    sm_alpha = solve_acceleration(my_alpha, w_motion, w_smooth);
    sm_beta  = solve_acceleration(my_beta , w_motion, w_smooth);        
    sm_gamma = solve_acceleration(my_gamma, w_motion, w_smooth);
    sm_Rs    = euler2matrix(sm_alpha, sm_beta, sm_gamma);
    
    % Smooth ts
    w_motion = WEIGHT_DATA;
    w_smooth = WEIGHT_SMOOTH_TRANSLATION;
    sm_ts1   = solve_acceleration( ts(:,1), w_motion, w_smooth );        
    sm_ts2   = solve_acceleration( ts(:,2), w_motion, w_smooth );        
    sm_ts3   = solve_acceleration( ts(:,3), w_motion, w_smooth );
    
    sm_ts = [sm_ts1, sm_ts2, sm_ts3];
    
    % Write output
    write_cameras(Ks, sm_Rs, sm_ts, output_smo_cam_file);
end