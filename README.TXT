==============
=== CREDIT ===
==============

ROS Wrapper for Posidonia Oceanica (P.O.) detection from bottom-looking under-
water images.

Author : Antoni Burguera
e-mail : antoni.burguera@uib.es

Original P.O. detector credit goes to:

Science   : A. Burguera, F. Bonin and J.L. Lisani
Code      : A. Burguera.
Contact   : antoni.burguera@uib.es
Reference :
A. Burguera, F. Bonin-Font, J.L. Lisani and A.B. Petro.
Towards Automatic Visual Sea Grass Detection in Underwater Areas of Ecological Interest.
In IEEE International Conference on Emerging Technologies and Factory Automation, Berlin, 2016.

=========================
=== BUILD THE PACKAGE ===
=========================

To build the package for the first time:

1) Remove all folders and files within this directory (if any), except "src" and
"README.TXT" (this file).
2) Remove src/CMakeLists.txt
3) Run (from within this directory) catkin_make

Further builds should only require catkin_make. Refer to ROS documentation for
more information on workspaces and building packages.

====================
=== RUN THE NODE ===
====================

The first step should be source devel/setup.bash from this directory. Unless
you source it from ~/.bashrc, source it every time a new bash session is
started.

The file src/pod/launch/pod_demo.launch is a valid roslaunch file. It runs the
POD node, plays a bagfile to feed it, and runs two image_view nodes to display
both the original image and the classified one.

However, it may have to be tuned as follows:

1) Change the topic remaps to the desired ones. There are two constraints:
  - They must be image topics.
  - The node POD expects a image_in topic and produces an image_out topic.

2) Change the args for the rosbag node to a path with the desired bag file.

3) Change the arguments for the pod node:
  - First argument is the path to the config directory. This is NOT related
    to ROS config. This path contains data that should not change. Thus,
    there should be no reason to change the first argument.
  - Second argument tells pod the output format. A value of 0 produces a
    black and white image in image_out, being P.O. labelled as black and
    non-P.O. labelled as white. A value of 1 overlays this info to the
    input image.

==========================
=== FULL USAGE EXAMPLE ===
==========================

Having the demo_launch.launch properly configured and the devel/setup.bash
sourced, just run the following command to see the demo in action:

roslaunch pod pod_demo.launch

===========================
=== TRAINING THE SYSTEM ===
===========================

There is no ROS wrapper for the trainer. So, training must be done with
the standalone version. Refer to the README.TXT file in the standalone
directory for more information.

After training, a .XML file is generated. To use this training data within the
ROS node, just place it in src/pod/config/ and name it TRAINEDSVM.XML

Note that if, for some reason, the config path has been changed (see previous
point "change arguments for the pod node"), TRAINEDSVM.XML must be placed in
that directory.

=======================
=== IMPORTANT NOTES ===
=======================

The file GABOR.CSV in the config path should NOT be modified. To place it in
another directory, be sure to change the config path data (see previous
sections) accordingly.

The pod behavior can be configured by means of the values in
src/pod/include/podefines.h. Refer to the standalone README.TXT for more
information.

Changing podefines.h requires re-building the node. Also, the standalone
trainer relies on another podefines.h file, with the same configuration
values. Be sure that you build and execute the trainer with the same values
in podefines.h than those used to build and execute the node. Otherwise,
the TRAINEDSVM.XML file might be invalid or lead to wrong classifications.

Also, note that these two podefines.h are NOT identical. So, do NOT copy the
file from the standalone trainer path to the node path.

Finally, take into account that there are minor changes in podescriber.h,
podescriber.cpp, podetector.h, podetector.cpp and podefines with respect
to the standalone versions. So, these files are NOT interchangeable between
the standalone version and the ROS wrapper version.

=======================
=== TROUBLESHOOTING ===
=======================

The thread library is used to speed-up the description process. This library
requires a c++11 compliant compiler. If not available or threads are unwanted
for some reason, just comment the following line in src/pod/include/podefines.h

#define _POD_THREADED_

Also, removing the c++11 flag from CMakeLists.txt might be necessary.