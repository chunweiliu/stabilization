#!bin/bash
name=$1
name1=${name}_l
name2=${name}_r
dir1=$name1/
dir2=$name2/
image_dir1=../dat/img/$dir1
image_dir2=../dat/img/$dir2
key_dir1=../log/txt/sift/$dir1
key_dir2=../log/txt/sift/$dir2
fund_key_dir1=../log/txt/sift_fund_matching/$dir1
fund_key_dir2=../log/txt/sift_fund_matching/$dir2
fund_mat_dir=../log/txt/fund_mat/$name/
tracks_dir=../log/txt/tracks_along/

# SIFT image detecting
<<COMMENT1
in_dir=$image_dir1
out_dir=$key_dir1
#if [ "$in_dir" == "" -o ! -d "$in_dir" ]; then
if [ ! -d "$in_dir" ]; then
    echo "The $in_dir is NOT exist in your system."
    exit 1
fi
if [ ! -d "$out_dir" ]; then
    echo "The $out_dir is NOT exist in your system."
    echo "mkdir $out_dir"
    mkdir $out_dir
fi
echo "../bin/image_sift $in_dir ../tmp/ $out_dir"
../bin/image_sift $in_dir ../tmp/ $out_dir

in_dir=$image_dir2;
out_dir=$key_dir2;
if [ ! -d "$in_dir" ]; then
    echo "The $in_dir is NOT exist in your system."
    exit 1
fi
if [ ! -d "$out_dir" ]; then
    echo "The $out_dir is NOT exist in your system."
    echo "mkdir $out_dir"
    mkdir $out_dir
fi
echo "../bin/image_sift $in_dir ../tmp/ $out_dir"
../bin/image_sift $in_dir ../tmp/ $out_dir

# SIFT image matching and find the fundamental matrix (left, right)
in_dir=$key_dir1;
out_dir=$fund_key_dir1;
if [ ! -d "$in_dir" ]; then
    echo "The $in_dir is NOT exist in your system."
    exit 1
fi
if [ ! -d "$out_dir" ]; then
    echo "The $out_dir is NOT exist in your system."
    echo "mkdir $out_dir"
    mkdir $out_dir
fi

in_dir=$key_dir2;
out_dir=$fund_key_dir2;
if [ ! -d "$in_dir" ]; then
    echo "The $in_dir is NOT exist in your system."
    exit 1
fi
if [ ! -d "$out_dir" ]; then
    echo "The $out_dir is NOT exist in your system."
    echo "mkdir $out_dir"
    mkdir $out_dir
fi

out_dir=$fund_mat_dir;
if [ ! -d "$out_dir" ]; then
    echo "The $out_dir is NOT exist in your system."
    echo "mkdir $out_dir"
    mkdir $out_dir
fi
echo "../bin/sift_match $key_dir1 $key_dir2 $fund_key_dir1 $fund_key_dir2 $fund_mat_dir"
../bin/sift_match $key_dir1 $key_dir2 $fund_key_dir1 $fund_key_dir2 $fund_mat_dir
COMMENT1

# SIFT keypoints tracking (left, right)
out_dir=$tracks_dir
if [ ! -d "$out_dir" ]; then
    echo "The $out_dir is NOT exist in your system."
    echo "mkdir $out_dir"
    mkdir $out_dir
fi

tracks_file1=$tracks_dir$name1.tracks
out_file=$tracks_file1
if [ ! -f "$out_file" ]; then
    echo "../bin/sift_tracking $image_dir1 $key_dir1 $out_file"
    ../bin/sift_tracking $image_dir1 $key_dir1 $out_file
fi
tracks_file2=$tracks_dir$name2.tracks
out_file=$tracks_file2
if [ ! -f "$out_file" ]; then
    echo "../bin/sift_tracking $image_dir2 $key_dir2 $out_file"
    ../bin/sift_tracking $image_dir2 $key_dir2 $out_file
fi
