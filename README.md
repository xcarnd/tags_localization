# Tags-based localization experiments

This is a repo for my codes of trying tags-based localization on my small drone.

## How will it work?

Essentially there's [Raspberry Pi Zero W](https://www.raspberrypi.org/products/raspberry-pi-zero-w/) with camera mounted on the drone (the on-board computer). It continuous captures image of an [Aruco](https://www.uco.es/investiga/grupos/ava/node/26) tags board and send to an off-board computer. The off-board computer then detect tags from the images and estimate camera pose w.r.t. the tags board. 

(Above is what has been done. Below is the plan.)

Later such pose will transformed into a relative movement to the drone's home location and then send back to the on-board computer, which will further send such information which will be utilized for sensor fusion to the autopilot (running PX4 stack).

## How to build

### Off-board part

The off-board part can be built as standard ROS package. Just run `catkin_make` at the workspace then everything should be fine.

### On-board part

Building the on-board part is more involved since I'm using Raspberry Pi Zero. OpenCV 3 & ROS has to be compiled from scratch on Raspberry Pi Zero, which will take significant amount of time. (Cross-compiling may come to the rescure, but I haven't tried yet.)

Some resources for compiling OpenCV & ROS:

1. https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/

2. http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

Besides OpenCV & ROS, [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) from Ubiquity Robitics is used for doing the actually capturing task since it can utilize the hardware acceleration of capturing video stream and image compression, which is important for using Raspberry Pi Zero as the on-board computer because the performance and WIFI bandwidth is quite limited. Guides for building `raspicam_node` can be found on the original repo as well as the one I forked (my fork only contains some camera settings specific to my raspi camera. Currently no further modifications).
