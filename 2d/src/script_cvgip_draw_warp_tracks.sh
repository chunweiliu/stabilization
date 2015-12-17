#!/bin/bash
name=$1
plot_length=$2

tracks_dir=../log/txt/tracks_along/
tracks_file=$tracks_dir${name}_l.tracks


# ORI
postfix=ori
image_dir=../log/png/cvgip/
image_name=${image_dir}${name}_feature_$postfix.png

path_dir=../log/txt/path_joint3d/
path_file=${path_dir}${name}_sim3d_0.5_l.path


output_dir=../log/png/cvgip/
output_original_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}_ori.png
output_smooth_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}.png

echo "../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length"
../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length


# 3D
postfix=3d
image_dir=../log/png/cvgip/
image_name=${image_dir}${name}_feature_$postfix.png

path_dir=../log/txt/path_joint3d/
path_file=${path_dir}${name}_sim3d_0.5_l.path


output_dir=../log/png/cvgip/
output_original_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}_ori.png
output_smooth_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}.png

echo "../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length"
../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length

# 2D
postfix=2d
image_dir=../log/png/cvgip/
image_name=${image_dir}${name}_feature_$postfix.png

path_dir=../log/txt/path_joint/
path_file=${path_dir}${name}_s+u_1e0_d+a_1e-1_l.path


output_dir=../log/png/cvgip/
output_original_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}_ori.png
output_smooth_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}.png

echo "../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length"
../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length

# ACTS
postfix=acts
image_dir=../log/png/cvgip/
image_name=${image_dir}${name}_feature_$postfix.png

path_dir=../log/txt/path_joint3d/
path_file=${path_dir}${name}_sim3d_0_l.path


output_dir=../log/png/cvgip/
output_original_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}_ori.png
output_smooth_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}.png

echo "../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length"
../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length

# KEZ
postfix=kez
image_dir=../log/png/cvgip/
image_name=${image_dir}${name}_feature_$postfix.png

path_dir=../log/txt/path_along/
path_file=${path_dir}${name}_s+u_1e1_l.path


output_dir=../log/png/cvgip/
output_original_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}_ori.png
output_smooth_file=${output_dir}${name}_feature_${postfix}_length_${plot_length}.png

echo "../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length"
../bin/draw_warp_tracks $image_name $tracks_file $path_file $output_original_file $output_smooth_file $plot_length
