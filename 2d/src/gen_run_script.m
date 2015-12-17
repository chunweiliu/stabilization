function gen_run_script( file_path )

%name_function = 'script_stable_test.sh';
    name_function = 'script_stereo_stabilize.sh';
    name_script = ['run_' name_function]
    fid = fopen( name_script, 'w');    
    fprintf(fid, '#!bin/bash\n');
    file_type = '*.avi';
    dats = dir( [file_path file_type] );
    for n = 1:2:length(dats)
	name = dats(n).name;
	name = name(1:end-6);

	cmd = ['sh ' name_function];
        fprintf(fid, '%s %s\n', cmd, name);
    end
    fclose(fid);

end
