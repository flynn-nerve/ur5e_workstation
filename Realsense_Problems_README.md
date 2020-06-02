# Instructions for solving issues where cameras will not work after installation or you are not able to install the required libraries

## Do not use these instructions unless you went through installation of the realsense cameras (on your own since the script will take care of this for you) and they are not working

If any libraries say that they cannot be installed, follow these instructions:

1. sudo apt-get remove librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
2. dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
3. sudo rm -f /etc/apt/sources.list.d/realsense-public.list
4. sudo apt-get update
5. sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

## Only follow these instructions if the previous solution also fails

*If this does not work, you should try to completely remove ROS from your system and start again*

Manually delete the librealsense and realsense packages

1. sudo apt-get remove librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
2. dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
3. sudo rm -f /etc/apt/sources.list.d/realsense-public.list
4. sudo apt-get remove --install-recommends linux-generic-lts-xenial xserver-xorg-core-lts-xenial xserver-xorg-lts-xenial xserver-xorg-video-all-lts-xenial xserver-xorg-input-all-lts-xenial libwayland-egl1-mesa-lts-xenial
5. sudo apt-get remove git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
6. sudo apt-get update && sudo apt-get upgrade
7. cd <your_ws>
8. rosdep install --from-paths src --ignore-src --rosdistro kinetic
9. catkin build
10. source ./devel/setup.bash

*This should solve your issues but you will most likely need to reinstall packages/dependencies as you go*


