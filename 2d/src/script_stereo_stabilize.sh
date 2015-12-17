#!/bin/bash
name=$1
name1=${name}_l
name2=${name}_r
dir1=$name1/
dir2=$name2/
image_dir1=../dat/img/$dir1
image_dir2=../dat/img/$dir2
key_dir1=../log/txt/sift_fund_matching/$dir1
key_dir2=../log/txt/sift_fund_matching/$dir2
tracks_dir=../log/txt/tracks_along/
path_dir=../log/txt/path_joint/
warp_dir=../log/png/warp_joint/
out_vid_dir=../out/warp_joint/

dump_log_dir=../log/txt/energy_function_joint/

path_merge_dir=../log/txt/path_merge/
merge_warp_dir=../log/png/warp_merge/
merge_out_dir=../out/warp_merge/

#for lambda_scale in 1e-1 1e0 1e1
for lambda_scale in 1e-1 
do
#for lambda_disparity in 1e1 1e0 1e-1
    for lambda_disparity in 1e-1
    do
    
	path_file1=$path_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_l.path
	path_file2=$path_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_r.path
    
	out_vid_file1=$out_vid_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_l.avi
	out_vid_file2=$out_vid_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_r.avi
	if [ ! -f "$out_vid_file1" -o ! -f "$out_vid_file2" ]; then
        

	    dump_log_file=$dump_log_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}.log
	
	    out_file1=$path_file1
	    out_file2=$path_file2
	    if [ ! -f "$out_file1" -o ! -f "$out_file2" ]; then

		tracks_file1=$tracks_dir$name1.tracks
		tracks_file2=$tracks_dir$name2.tracks
		echo "../bin/stabilize_estimate $key_dir1 $key_dir2 $tracks_file1 $tracks_file2 $out_file1 $out_file2 $lambda_scale $lambda_scale $lambda_disparity $lambda_disparity"
		../bin/stabilize_estimate $key_dir1 $key_dir2 $tracks_file1 $tracks_file2 $out_file1 $out_file2 $lambda_scale $lambda_scale $lambda_disparity $lambda_disparity > $dump_log_file
	    fi

	    out_dir=$warp_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_l/
	    if [ ! -d "$out_dir" ]; then
		echo "The $out_dir is NOT exist in your system."
		echo "mkdir $out_dir"
		mkdir $out_dir
	    fi
	    echo "../bin/image_warp $image_dir1 $path_file1 $out_dir"
	    ../bin/image_warp $image_dir1 $path_file1 $out_dir

	    echo "ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file1"
	    ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file1
	
	    out_dir=$warp_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_r/
	    if [ ! -d "$out_dir" ]; then
		echo "The $out_dir is NOT exist in your system."
		echo "mkdir $out_dir"
		mkdir $out_dir
	    fi
	    echo "../bin/image_warp $image_dir2 $path_file2 $out_dir"
	    ../bin/image_warp $image_dir2 $path_file2 $out_dir

	    echo "ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file2"
	    ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file2
	fi

	out_vid_merge1=$merge_out_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_merge_l.avi
	out_vid_merge2=$merge_out_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_merge_r.avi
	if [ ! -f "$out_vid_merge1" -o ! -f "$out_vid_merge2" ]; then
	
	    path_merge=$path_merge_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_merge.path
	    if [ ! -f "$path_merge" ]; then
		echo "../bin/merge_path $path_file1 $path_file2 > $path_merge"
		../bin/merge_path $path_file1 $path_file2 > $path_merge
	    fi
	
	    out_dir=$merge_warp_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_merge_l/
	    if [ ! -d "$out_dir" ]; then
		echo "The $out_dir is NOT exist in your system."
		echo "mkdir $out_dir"
		mkdir $out_dir
	    fi
	    echo "../bin/image_warp $image_dir1 $path_merge $out_dir"
	    ../bin/image_warp $image_dir1 $path_merge $out_dir

	    echo "ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_merge1"
	    ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_merge1

	    out_dir=$merge_warp_dir${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_merge_r/
	    if [ ! -d "$out_dir" ]; then
		echo "The $out_dir is NOT exist in your system."
		echo "mkdir $out_dir"
		mkdir $out_dir
	    fi
	    echo "../bin/image_warp $image_dir2 $path_merge $out_dir"
	    ../bin/image_warp $image_dir2 $path_merge $out_dir

	    echo "ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_merge2"
	    ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_merge2
	fi
    done
done
