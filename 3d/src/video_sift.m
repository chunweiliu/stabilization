%VIDEO_SIFT find sift feature by rand generate sift file
% input:
%  video_dir
%  output_sift_dir
function video_sift(video_dir, output_sift_dir)

    % Check if output directory exist
    if (~exist(output_sift_dir, 'dir'))
        cmd = ['mkdir ' output_sift_dir];
        disp(cmd); 
        eval(cmd);
    end

    % Give a rand num to run sift parallel
    r = rand(1);
    tmppgm = sprintf('../tmp/%s.pgm', r);

    %fprintf(1, 'video_sift: 0.00');
    dats = dir([video_dir '*.jpg']);
    for n = 1:length(dats)
        %fprintf(1, '\b\b\b\b%.2f', n/length(dats));
        
        output_file = sprintf('%s%04d.key', output_sift_dir, n-1);
        if exist(output_file, 'file'), continue; end
        
        
        % Load image
        imageFile = sprintf('%s%04d.jpg', video_dir, n-1);
        image = imread(imageFile);

        % If you have the Image Processing Toolbox, you can uncomment the following
        %   lines to allow input of color images, which will be converted to grayscale.
        % if isrgb(image)
        image = rgb2gray(image);
        % end

        [rows, cols] = size(image); 
        
        % Convert into PGM imagefile, readable by "keypoints" executable
        f = fopen(tmppgm, 'w');
        if f == -1
            error('Could not create file tmp.pgm.');
        end
        fprintf(f, 'P5\n%d\n%d\n255\n', cols, rows);
        fwrite(f, image', 'uint8');
        fclose(f);
        
        % Call keypoints executable
        if isunix
            command = '!../bin/sift ';
        else
            command = '!..\bin\siftWin32 ';
        end
        command = [command ' < ' tmppgm ' > ' output_file];
        eval(command);
        delete(tmppgm);
        
    end
    %fprintf(1, '\n');
    

end

