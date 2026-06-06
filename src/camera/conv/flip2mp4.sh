ffmpeg -r 30 -i $1 -vf "transpose=2,transpose=2" -c:v libx264 -pix_fmt yuv420p $1.mp4
