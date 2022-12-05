#!/bin/bash
ffmpeg -i $1 -vf scale=$2 -c:v libvpx-vp9 -b:v 1M -pass 1 -an -f null /dev/null 
ffmpeg -i $1 -vf scale=$2 -c:v libvpx-vp9 -b:v 1M -pass 2 $3
