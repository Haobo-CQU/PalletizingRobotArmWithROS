# PalletizingRobotArmWithROS
Control a robot arm to grasp and palletize objects automatically with ROS and OpenCV

Used Name: RGB Blocks Classification Using Robot Arm(uARM Swift Pro) with ROS and OpenCV
Video Link: https://youtu.be/hYwxyzRh7vU


# Brief Description
The robot arm is modified based on uARM Swift Pro with 3D printout.

The code is used for educational competition based on ROS.

Code is written with C++ and python. Some of them are cmake and bash used for ROS environment.


# Usage (Competition Rule)
There are 3 colors(RGB) * 3 shapes (cube,cuboid,cylinder) = 9 Blocks, in front of the robot arm.

I am supposed to classify them, pick them up and lay dowm on the left side automatically with some order:

Same shapes should be lay together with sequence: form top to bottom —— Red，Blue，Green.

**The Programma is required to run within ROS.**


# Author
Haobo Yuan, Chongqing University, China.

yuanhb0521 AT gmail DOT com


# Env info
## Environment：
- Windows10 PC
    - VMvare 15
    - Ubuntu 16.04
    - ROS Kinetic
    - Python 2.7
    - OpenCV 2.4.9.1

- UFACTORY uARM Swift Pro Robot ARM

- Arduino and its IDE (I use Mega2560. However, uno is acceptable)

## Based lib & blog & Package:


- uArm-Developer / RosForSwiftAndSwiftPro
    > https://github.com/uArm-Developer/RosForSwiftAndSwiftPro
    > https://www.ufactory.cc/#/cn/support/download/pro

- rosserial_python
    > http://wiki.ros.org/rosserial_python?distro=kinetic

- usb_cam
- image_proc
- (camera_calibration)
- OpenCV
    - Sorry about lack info about OpenCV since this is not my responsible part. I know short of it.

# File Catalog
- readme.md
- (a Solidworks part Screen-shot).jpg
- (a show video).mp4
- code
    - (OpenCV test photo).jpg
    - common_command.md
    - ros_arm
        - arduino_code (written myself)
        - arm_mian (written myself)
        - swiftpro (git clone)
        - pro_moveit_config (git clone)
