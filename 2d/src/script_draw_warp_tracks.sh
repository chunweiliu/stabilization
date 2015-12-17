#!/bin/bash
name=$1
lambda_scale=$2
lambda_disparity=$3
plot_length=$4

#name1=${name}_l
#name2=${name}_r
#dir1=$name1/
#dir2=$name2/
#image_dir1=../dat/img/$dir1
#image_dir2=../dat/img/$dir2

tracks_dir=../log/txt/tracks_along/
tracks_file1=$tracks_dir${name}_l.tracks
tracks_file2=$tracks_dir${name}_r.tracks

path_dir=../log/txt/path_joint/
path_file1=${path_dir}${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_l.path
path_file2=${path_dir}${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_r.path

output_dir=../log/png/tracks/
output_original_file1=${output_dir}${name}_length_${plot_length}_l.png
output_original_file2=${output_dir}${name}_length_${plot_length}_r.png
output_smooth_file1=${output_dir}${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_length_${plot_length}_l.png
output_smooth_file2=${output_dir}${name}_s+u_${lambda_scale}_d+a_${lambda_disparity}_length_${plot_length}_r.png

image_dir=../log/png/gray_anaglyph/csiegirl/
image_name1=${image_dir}csiegirl_0163.png
image_name2=${image_dir}csiegirl_proposed_0163.png

echo "../bin/draw_warp_tracks $image_name1 $tracks_file1 $path_file1 $output_original_file1 $output_smooth_file1 $plot_length"
../bin/draw_warp_tracks $image_name1 $tracks_file1 $path_file1 $output_original_file1 $output_smooth_file1 $plot_length

echo "../bin/draw_warp_tracks $image_name2 $tracks_file2 $path_file2 $output_original_file2 $output_smooth_file2 $plot_length"
../bin/draw_warp_tracks $image_name2 $tracks_file2 $path_file2 $output_original_file2 $output_smooth_file2 $plot_length 
