This file is intended to help Jules (or any other AI agent) complete tasks associated with this repository.

This repository contains a SLAM library called ORB_SLAM3. It is a heavily modified fork.

This builds and runs on Ubuntu 24.04. Here are the build and test instructions:

~~~
mkdir build
cd build
cmake ..
cmake --build . -j $NPROC
ctest
~~~

Ignore all refences to Pangolin. Pangolin is not used in this fork.

Use googletest for testing.

These directories and files are not used or out of date. They can be ignored:
* evaluation
* Examples
* Examples_old
* build.sh
* build_ros.sh
* Calibration_Tutorial.pdf
* README.md
