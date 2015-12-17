#!bin/bash
name=$1

sh script_find_tracks.sh $name
sh script_find_tracks_along.sh $name
sh script_stable_test.sh $name
sh script_stereo_stabilize.sh $name
