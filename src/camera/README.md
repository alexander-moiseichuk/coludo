# camera recoding support

It seems Luckfox Pico Mini and sc3336 can support out of box decent 2304x1296 30 FPS experience for small weight:
- board + 64 GB card
- camera
- power boost
- battery (3.7 1 cell LiPo) + 5V booster
- all together weight 30 gramms

The 32GB card or less will be sufficient but on launch pad missie may stay 15 minutes or more, so avoid losing
launch experience it must recorded in format which allows to survice for all possible frames. In top resolution
only mpeg works fine as VI channel has enought memory. The load about 30% and temperature hits acceptable 55-60C
without any passive cooling. Videos writing into /userdata/videos (see opt/camera/camerad.conf), so this area nice
to have 10+ GB as 15 mins default video consumes ~3.5GB

See sc3336.txt for possible formats but for fast implementation the existing framework is used from /oem/usr/bin:
- rkaiq_3A_server supports proper channel setup (not green, not dark)
- simple_vi_bind_venc is a nice tool to write in mjpeg (save until last frame) for ~15 mins clips (~3.5GB) with 30 FPS

You can choose (see opt/camera/camerad.conf) h264 or h265 but you have to reduce resoltution or increase CMA size in kernel
over default 24MB. Now for 64MB board 15 MB stil available and not much space to play around, so mjpeg is simply fits out of box.

# installation

push files to device, e.g.

	$ adb push opt/* /opt/

link in startup scripts e.g. 
```
	# adb shell
	# ln -s /opt/camera/camerad.sh /etc/init.d/S99camerad
	# ln -s /opt/hwclock/hwclockd.sh /etc/init.d/S20hwclockd      <--- this one optional
	# chmod -x $(which rkipc)                                     <--- it consumes a lot of cycles unnecessary
        # update date to current if you need
        # sync
        # poweroff
```

if started from USB-power supply (not PC+rndis) it will start writing files /userdata/videos/000-2304x1296.mjpeg etc.
to indicate it during boot LED does ~5s ON period and then start actively blinking following mmc1 activity,
otherwise turned off.

in case of PC script should not start allowing download videos in easy way and LED is OFF.
