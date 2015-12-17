% RUN_SCRIPT processing a video go throught all steps
% Requires:
%   camera_l_file, camera_r_file, scenes_l_file, scenes_r_file
%   estimating by ACTS
function run_script(vidname)

    % Parameter
    %WEIGHT_STEREO    = 0.5;
    WEIGHT_STEREO    = 1.0;
    GRID_ROW_NUM     = 15;
    GRID_COL_NUM     = 20;
    SMOOTHNESS_ALPHA = 1;
    %SMOOTHNESS_ALPHA = 7;
    %SMOOTHNESS_ALPHA = 20;
    %SMOOTHNESS_ALPHA = 100;

    % Check SFM results
    % Find 3D smooth cameras
    camera_l_file     = sprintf('../log/cameras/%s_l.cameras', vidname);
    camera_r_file     = sprintf('../log/cameras/%s_r.cameras', vidname);
    % Reproject 3D shaky scene and 3D smooth scene to 2D
    scenes_l_file = sprintf('../log/scenes/%s_l.scenes', vidname);
    scenes_r_file = sprintf('../log/scenes/%s_r.scenes', vidname);
    if ~exist(camera_l_file, 'file') || ~exist(camera_r_file, 'file') || ...
       ~exist(scenes_l_file, 'file') || ~exist(scenes_r_file, 'file')
       fprintf(1, 'The camera or scene file of %s may not exist.\n',...
           vidname)
       return;
    end
    
    % Check frame number
    video_l_dir = sprintf('../dat/img/%s_l/', vidname);
    video_r_dir = sprintf('../dat/img/%s_r/', vidname);
    dats = dir([video_l_dir '*.jpg']);
    if length(dats) ~= length(dir([video_r_dir '*.jpg']))
        fprintf(1,'Frame number between %s and %s are not equal\n', ...
            video_l_dir, video_r_dir);
        return
    end
    if isempty(dats) || isempty(length(dir([video_r_dir '*.jpg'])))
        fprintf(1,'Video %s or %s may not exist\n', ...
            video_l_dir, video_r_dir);
        return
    end
    vidframe = length(dats);
    im = imread([video_l_dir dats(1).name]);
    [frame_height, frame_width, frame_channel] = size(im);
    
    % Find SIFT features
    key_l_dir   = sprintf('../log/key/%s_l/', vidname);
    key_r_dir   = sprintf('../log/key/%s_r/', vidname);
    
    video_sift(video_l_dir, key_l_dir);
    video_sift(video_r_dir, key_r_dir);
    
    % Match SIFT features
    match_dir = sprintf('../log/matching/%s/', vidname);
    video_match(key_l_dir, key_r_dir, match_dir);
    
    % ACTS run strcture from motion, get left and right cameras and scenes
    
    % Find 3D smooth cameras
    %camera_l_file     = sprintf('../log/cameras/%s_l.cameras', vidname);
    %camera_r_file     = sprintf('../log/cameras/%s_r.cameras', vidname);
    
    camera_l_smo_file = sprintf('../log/cameras/%s_l_smo.cameras', vidname);
    camera_r_smo_file = sprintf('../log/cameras/%s_r_smo.cameras', vidname); 
    
    smooth_cameras(camera_l_file, camera_l_smo_file);
    smooth_cameras(camera_r_file, camera_r_smo_file);
    
    % Reproject 3D shaky scene and 3D smooth scene to 2D
    %scenes_l_file = sprintf('../log/scenes/%s_l.scenes', vidname);
    %scenes_r_file = sprintf('../log/scenes/%s_r.scenes', vidname);
    
    points_l_dir  = sprintf('../log/points/%s_l/', vidname);
    points_r_dir  = sprintf('../log/points/%s_r/', vidname);
    
    %reproject_3d_scenes_to_2d(camera_l_file, scenes_l_file, points_l_dir);
    %reproject_3d_scenes_to_2d(camera_r_file, scenes_r_file, points_r_dir);
        
    points_l_smo_dir  = sprintf('../log/points/%s_l_smooth/', vidname);
    points_r_smo_dir  = sprintf('../log/points/%s_r_smooth/', vidname);
    
    reproject_3d_scenes_to_2d(camera_l_file, scenes_l_file, ...
        points_l_dir, camera_l_smo_file, points_l_smo_dir, ...
        frame_width, frame_height);
    reproject_3d_scenes_to_2d(camera_r_file, scenes_r_file, ...
        points_r_dir, camera_r_smo_file, points_r_smo_dir, ...
        frame_width, frame_height);
    
    %reproject_3d_scenes_to_2d(camera_l_smo_file, scenes_l_file, points_l_dir);
    %reproject_3d_scenes_to_2d(camera_l_smo_file, scenes_l_file, points_l_dir);    
    
    % Find similarity transform with stereoscopic constraint, and also
    % warp the output frame.
    output_matching_l_dir = sprintf('../log/matching/%s_l_p3d_ws%s/',...
                            vidname, num2str(WEIGHT_STEREO));
    output_matching_r_dir = sprintf('../log/matching/%s_r_p3d_ws%s/',...
                            vidname, num2str(WEIGHT_STEREO));
    output_path_l_file    = sprintf('../log/warp/%s_l_p3d_ws%s.path', ...
                            vidname, num2str(WEIGHT_STEREO));
    output_path_r_file    = sprintf('../log/warp/%s_r_p3d_ws%s.path', ...
                            vidname, num2str(WEIGHT_STEREO));
    output_video_l_dir    = sprintf('../out/%s_l_p3d_ws%s/', vidname, num2str(WEIGHT_STEREO));
    output_video_r_dir    = sprintf('../out/%s_r_p3d_ws%s/', vidname, num2str(WEIGHT_STEREO));
    
    %video_find_stereo_similarity(vidname, vidframe, WEIGHT_STEREO,...
    %     output_path_l_file, output_path_r_file);
    video_find_stereo_similarity(vidname, vidframe, WEIGHT_STEREO,...
         output_matching_l_dir, output_matching_r_dir, ...
         output_path_l_file, output_path_r_file, ...
         output_video_l_dir, output_video_r_dir);
     
    % Apply content preserving warp (c implementation)
    %{
    output_video_l_dir    = sprintf('../out/%s_l_p3d_ws%s_cpw%sx%s+%s/', ...
                            vidname, num2str(WEIGHT_STEREO), num2str(GRID_ROW_NUM),...
                            num2str(GRID_COL_NUM), num2str(SMOOTHNESS_ALPHA));    
    video_content_preserving_warps(video_l_dir, output_matching_l_dir, ...
        output_video_l_dir, vidframe-1, frame_width, frame_height,...);
        GRID_ROW_NUM, GRID_COL_NUM, SMOOTHNESS_ALPHA);
    
    output_video_r_dir    = sprintf('../out/%s_r_p3d_ws%s_cpw%sx%s+%s/', ...
                            vidname, num2str(WEIGHT_STEREO), num2str(GRID_ROW_NUM),...
                            num2str(GRID_COL_NUM), num2str(SMOOTHNESS_ALPHA));
    video_content_preserving_warps(video_r_dir, output_matching_r_dir, ...
        output_video_r_dir, vidframe-1, frame_width, frame_height,...);
        GRID_ROW_NUM, GRID_COL_NUM, SMOOTHNESS_ALPHA);
    %}
end