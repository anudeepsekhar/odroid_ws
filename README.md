# odroid_ws

```
git clone https://github.com/TEAMIFOR/odroid_ws.git
cd odroid_ws
./launcher.sh
```

## Changelog
v1 single rospackage to run ellipse detection(python) and offboard demo(cpp) together

##Notes
default image input camera at /dev/video0, can be changed inside usb_cam launchfile, also to be changed under missionpkg launchfile
calibrate camera first (rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/usb_cam/raw_image camera:=/usb_cam

