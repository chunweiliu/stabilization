% WRITE_CAMERAS write cameras as ACTS's output format
function write_cameras( Ks, Rs, ts, output_cam_file )
    num_frame = size(Rs, 1);
    fid = fopen( output_cam_file, 'w' );
    
    % give 4 comment line like original camear file
    fprintf( fid, '#\n' );
    fprintf( fid, '#\n' );
    fprintf( fid, '#\n' );
    fprintf( fid, '#\n\n' );
    
    fprintf( fid, '%d\n\n', num_frame );
    
    for n = 1:num_frame
       print_matrix( fid, Ks(n,:));
       print_matrix( fid, Rs(n,:));
       fprintf( fid, '%f %f %f\n', ts(n, 1), ts(n, 2), ts(n, 3));
       fprintf( fid, '\n\n' );
    end
    fclose(fid);
end

function print_matrix( fid, v )
    fprintf( fid, '%f %f %f\n', v(1), v(2), v(3) );
    fprintf( fid, '%f %f %f\n', v(4), v(5), v(6) );
    fprintf( fid, '%f %f %f\n', v(7), v(8), v(9) );
end