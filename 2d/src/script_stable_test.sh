#!/bin/bash
name=$1
name1=${name}_l
name2=${name}_r
dir1=$name1/
dir2=$name2/
image_dir1=../dat/img/$dir1
image_dir2=../dat/img/$dir2
tracks_dir=../log/txt/tracks_along/
path_dir=../log/txt/path_along/
warp_dir=../log/png/warp_along/
out_vid_dir=../out/warp_along/
draw_tracks_dir=../log/png/tracks/
draw_tracks_len=30

dump_log_dir=../log/txt/energy_function_along/

path_merge_dir=../log/txt/path_merge/
merge_warp_dir=../log/png/warp_merge/
merge_out_dir=../out/warp_merge/


#for para in 1e0 1e1 1e2
for para in 1e0
do
    
    path_file1=$path_dir${name}_s+u_${para}_l.path
    path_file2=$path_dir${name}_s+u_${para}_r.path

    out_vid_file=$out_vid_dir${name}_s+u_${para}_l.avi
    if [ ! -f "$out_vid_file" ]; then

	tracks_file1=$tracks_dir$name1.tracks
	dump_log_file1=$dump_log_dir${name}_s+u_${para}_l.log

	out_file=$path_file1
	if [ ! -f "$out_file" ]; then
	    echo "../bin/kez_stabilize_estimate $tracks_file1 $out_file $para $para"
	    ../bin/kez_stabilize_estimate $tracks_file1 $out_file $para $para > $dump_log_file1
	fi

<<COMMENT_DRAW1
	draw_tracks_file1=${draw_tracks_dir}${name}_s+u_${para}_length_${draw_tracks_len}_l.png
	draw_smooth_file1=${draw_tracks_dir}${name}_s+u_${para}_length_${draw_tracks_len}W_l.png
	
	out_file=$draw_tracks_file1
	if [ ! -f "$out_file" ]; then
	    echo "../bin/draw_warp_tracks $image_dir1 $tracks_file1 $path_file1 $draw_tracks_file1 $draw_smooth_file1 $draw_tracks_len"
	    ../bin/draw_warp_tracks $image_dir1 $tracks_file1 $path_file1 $draw_tracks_file1 $draw_smooth_file1 $draw_tracks_len
	fi
COMMENT_DRAW1

	out_dir=$warp_dir${name}_s+u_${para}_l/
	if [ ! -d "$out_dir" ]; then
	    echo "The $out_dir is NOT exist in your system."
	    echo "mkdir $out_dir"
	    mkdir $out_dir
	fi
	echo "../bin/image_warp $image_dir1 $path_file1 $out_dir"
	../bin/image_warp $image_dir1 $path_file1 $out_dir

    	echo "ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file"
	ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file
    fi

    out_vid_file=$out_vid_dir${name}_s+u_${para}_r.avi
    if [ ! -f "$out_vid_file" ]; then

	tracks_file2=$tracks_dir$name2.tracks
	dump_log_file2=$dump_log_dir${name}_s+u_${para}_r.log

	out_file=$path_file2
	if [ ! -f "$out_file" ]; then
	    echo "../bin/kez_stabilize_estimate $tracks_file2 $out_file $para $para"
	    ../bin/kez_stabilize_estimate $tracks_file2 $out_file $para $para > $dump_log_file2
	fi

<<COMMENT_DRAW2
	draw_tracks_file2=${draw_tracks_dir}${name}_s+u_${para}_length_${draw_tracks_len}_r.png
	draw_smooth_file2=${draw_tracks_dir}${name}_s+u_${para}_length_${draw_tracks_len}W_r.png

	out_file=$draw_tracks_file2
	if [ ! -f "$out_file" ]; then
	    echo "../bin/draw_warp_tracks $image_dir2 $tracks_file2 $path_file2 $draw_tracks_file2 $draw_smooth_file2 $draw_tracks_len"
	    ../bin/draw_warp_tracks $image_dir2 $tracks_file2 $path_file2 $draw_tracks_file2 $draw_smooth_file2 $draw_tracks_len
	fi
COMMENT_DRAW2

	out_dir=$warp_dir${name}_s+u_${para}_r/
	if [ ! -d "$out_dir" ]; then
	    echo "The $out_dir is NOT exist in your system."
	    echo "mkdir $out_dir"
	    mkdir $out_dir
	fi
	echo "../bin/image_warp $image_dir2 $path_file2 $out_dir"
	../bin/image_warp $image_dir2 $path_file2 $out_dir

	echo "ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file"
	ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file
    fi

    out_vid_merge1=$merge_out_dir${name}_s+u_${para}_merge_l.avi
    out_vid_merge2=$merge_out_dir${name}_s+u_${para}_merge_r.avi
    if [ ! -f "$out_vid_merge1" -o ! -f "$out_vid_merge2" ]; then
	
	path_merge=$path_merge_dir${name}_s+u_${para}_merge.path
	if [ ! -f "$path_merge" ]; then
	    echo "../bin/merge_path $path_file1 $path_file2 > $path_merge"
	    ../bin/merge_path $path_file1 $path_file2 > $path_merge
	fi
	
	out_dir=$merge_warp_dir${name}_s+u_${para}_merge_l/
	if [ ! -d "$out_dir" ]; then
	    echo "The $out_dir is NOT exist in your system."
	    echo "mkdir $out_dir"
	    mkdir $out_dir
	fi

	echo "../bin/image_warp $image_dir1 $path_merge $out_dir"
	../bin/image_warp $image_dir1 $path_merge $out_dir
	echo "ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_merge1"
	ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_merge1

	out_dir=$merge_warp_dir${name}_s+u_${para}_merge_r/
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
