ffmpeg -r 30 -i $1 -vf "vflip" -c:v libx264 -pix_fmt yuv420p $1.mp4
