#!bin/bash
name=$1
path_type=$2

name1=${name}_l
name2=${name}_r
dir1=$name1/
dir2=$name2/
image_dir1=../dat/img/$dir1
image_dir2=../dat/img/$dir2

path_dir=../log/txt/$path_type/
path_merge_dir=../log/txt/path_merge/

out_image_dir=../log/png/warp_merge/
out_vid_dir=../out/warp_merge/

if [ $path_type = "path_joint" ]; then

#for para1 in 1e0 1e1 1e2
    for para1 in 1e1
    do
#for para2 in 1e-6 1e1-3 1e0 1e3
	for para2 in 1e-6
	do    
	    path_file1=$path_dir${name}_s+u_${para1}_d+a_${para2}_l.path
	    path_file2=$path_dir${name}_s+u_${para1}_d+a_${para2}_r.path
	    out_vid_file1=$out_vid_dir${name}_s+u_${para1}_d+a_${para2}_merge_${path_type}_l.avi
	    out_vid_file2=$out_vid_dir${name}_s+u_${para1}_d+a_${para2}_merge_${path_type}_r.avi
	    path_merge_file=$path_merge_dir${name}_s+u_${para1}_d+a_${para2}_merge_${path_type}.path

	    if [ ! -f "$path_file1" -o ! -f "$path_file2" ]; then
		continue
	    fi

	    if [ ! -f "$out_vid_file1" -o ! -f "$out_vid_file2" ]; then

		if [ ! -f "$path_merge_file" ]; then
		    echo "../bin/merge_path $path_file1 $path_file2 > $path_merge_file"
		    ../bin/merge_path $path_file1 $path_file2 > $path_merge_file
		fi

		out_dir=$out_image_dir${name}_s+u_${para1}_d+a_${para2}_l/
		if [ ! -d "$out_dir" ]; then
		    echo "The $out_dir is NOT exist in your system."
		    echo "mkdir $out_dir"
		    mkdir $out_dir
		fi
		echo "../bin/image_warp $image_dir1 $path_merge_file $out_dir"
		../bin/image_warp $image_dir1 $path_merge_file $out_dir

		echo "ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file1"
		ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file1

		out_dir=$out_image_dir${name}_s+u_${para1}_d+a_${para2}_r/
		if [ ! -d "$out_dir" ]; then
		    echo "The $out_dir is NOT exist in your system."
		    echo "mkdir $out_dir"
		    mkdir $out_dir
		fi

		echo "../bin/image_warp $image_dir2 $path_merge_file $out_dir"
		../bin/image_warp $image_dir2 $path_merge_file $out_dir

		echo "ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file2"
		ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file2

	    fi

	done
    done
elif [ $path_type = "path_along" ]; then

    for para1 in 1e1
    do
	path_file1=$path_dir${name}_s+u_${para1}_l.path
	path_file2=$path_dir${name}_s+u_${para1}_r.path
	out_vid_file1=$out_vid_dir${name}_s+u_${para1}_merge_${path_type}_l.avi
	out_vid_file2=$out_vid_dir${name}_s+u_${para1}_merge_${path_type}_r.avi
	path_merge_file=$path_merge_dir${name}_s+u_${para1}_d+a_${para2}_merge_${path_type}.path
	
	if [ ! -f "$path_file1" -o ! -f "$path_file2" ]; then
	    continue
	fi

	if [ ! -f "$out_vid_file1" -o ! -f "$out_vid_file2" ]; then

	    if [ ! -f "$path_merge_file" ]; then
		echo "../bin/merge_path $path_file1 $path_file2 > $path_merge_file"
	        ../bin/merge_path $path_file1 $path_file2 > $path_merge_file
	    fi

	    out_dir=$out_image_dir${name}_s+u_${para1}_l/
	    if [ ! -d "$out_dir" ]; then
	        echo "The $out_dir is NOT exist in your system."
	        echo "mkdir $out_dir"
	        mkdir $out_dir
	    fi
	    echo "../bin/image_warp $image_dir1 $path_merge_file $out_dir"
	    ../bin/image_warp $image_dir1 $path_merge_file $out_dir

	    echo "ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file1"
	    ffmpeg -i ${out_dir}${name}_%04d_l.png -vcodec mpeg1video -qscale 0.6 $out_vid_file1

	    out_dir=$out_image_dir${name}_s+u_${para1}_r/
	    if [ ! -d "$out_dir" ]; then
	        echo "The $out_dir is NOT exist in your system."
	        echo "mkdir $out_dir"
	        mkdir $out_dir
	    fi

	    echo "../bin/image_warp $image_dir2 $path_merge_file $out_dir"
	    ../bin/image_warp $image_dir2 $path_merge_file $out_dir

	    echo "ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file2"
	    ffmpeg -i ${out_dir}${name}_%04d_r.png -vcodec mpeg1video -qscale 0.6 $out_vid_file2

	fi

    done
fi
