% VIDEO_CONTENT_PRESERVING_WARPS script call content-preserving warps
% input
%  video_dir,          c x 1,           input_original images
%  input_matching_dir, c x 1,           [x y smoothx smoothy] %04d.matching
%  output_video_dir,   c x 1,           output warped image
%  vidframe,           1 x 1,           video frame number
%  GRID_ROW_NUM        1 x 1,           grids: GRID_ROW_NUM x GRID_COL_NUM
%  GRID_COL_NUM        1 x 1,
%  SMOOTHNESS_ALPHA    1 x 1,           paper suggestion 20
function video_content_preserving_warps(video_dir, input_matching_dir, ...
        output_video_dir, vidframe, video_width, video_height, ...);
        GRID_ROW_NUM, GRID_COL_NUM, SMOOTHNESS_ALPHA)

    if isunix
        error('video_content_preserving_warps: The binary can only use in Windows');
    else
        binary = '!..\bin\warp';
        %binary = '!..\bin\warp_grid';
    end
    
    % Check if output directory exist
    if (~exist(output_video_dir, 'dir'))
        cmd = ['mkdir ' output_video_dir];
        disp(cmd); 
        eval(cmd);
    end
    
    command = sprintf('%s %s %s %s %s %s %s %s', binary,...
        video_dir, input_matching_dir, output_video_dir, ...
        num2str(vidframe), num2str(video_width), num2str(video_height), ...
        num2str(GRID_ROW_NUM), num2str(GRID_COL_NUM), num2str(SMOOTHNESS_ALPHA));
    disp(command);
    eval(command);
end

