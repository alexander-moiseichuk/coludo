ffmpeg -r 30 -i $1 -vf "transpose=2,transpose=2" -c:v mjpeg -q:v 3 $1.avi
