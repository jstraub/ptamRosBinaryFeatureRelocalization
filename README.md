PTAM-GPL with ROS Integration and Binary Feature Relocalization
===============================================================

This project adds two features to the original PTAM-GPL (https://github.com/Oxford-PTAM/PTAM-GPL.git) release:
- Integration of a fast relocalization mechanism based on binary features as described in the following paper:
  *Fast Relocalization For Visual Odometry Using Binary Features* (Julian Straub, Sebastian Hilsenbeck, Georg Schroth, Robert Huitl, Andreas Möller, Eckehard Steinbach), In IEEE International Conference on Image Processing (ICIP), 2013. 
  Download from http://www.jstraub.de/download/straub2013fastRelocalization.pdf
- *Integration with the Robot Operating System (ROS)* http://www.ros.org/. This among other features allows replaying of recorded videos as rosbags.

What is PTAM
------------

PTAM is a monocular SLAM (Simultaneous Localization and Mapping) system useful for real-time
6-DOF camera tracking in small scenes. It was originally developed as a research system in the Active 
Vision Laboratory of the University of Oxford, as described in the following papers:

- Georg Klein and David Murray, "Parallel Tracking and Mapping for Small AR Workspaces", Proc. ISMAR 2007
- Georg Klein and David Murray, "Improving the Agility of Keyframe-based SLAM", Proc. ECCV 2008

PTAM (Parallel Tracking and Mapping) released under GPLv3 and was obtain from https://github.com/Oxford-PTAM/PTAM-GPL.git
see http://www.robots.ox.ac.uk/~gk/PTAM/ for version history (this code uses the GPL release of version v1.0-r114)


Installation of Necessary 3rd-party Code
----------------------------------------

- install gvars3, libcvd and TooN (as described in PTAM install) on the same folder level as ptamBriefRelocalizeStandalone i.e.:
  ```
   ./
   ./3rdparty/gvars3/
   ./3rdparty/libcvd/
   ./3rdparty/TooN/
   ./ptamBriefRelocalizeStandalone/
  ```
   it is sufficient to install them in place

- gvars3: http://www.edwardrosten.com/cvd/gvars3.html
- libcvd: http://www.edwardrosten.com/cvd/index.html
- TooN: http://www.edwardrosten.com/cvd/toon.html

- the code should work with ros fuerte and groovy (you need to change the include and link directories in the CMakeLists.txt if you want to use the older ros fuerte)

Compiling PTAM-GPL with ROS Integration
---------------------------------------

- export NAVVIS_WORKSPACE=/path/to/parent/folder/ to point to the parent folder of ptamBriefRelocalizeStandalone and 3rdparty
- Compile the roswrapper: 
  ```
  cd PTAM; 
  ./makeRos.sh; 
  cd ../
  ```
- Compile PTAM itself: 
  ```
  cd PTAM/build; 
  cmake ..; 
  make install; 
  cd ../../;
  ```
- see ./CompilationNotes.txt for more information about compiling PTAM

Running PTAM-GPL with Binary Feature Relocalization
---------------------------------------------------

- set the LD_LIBRARY_PATH
  ```
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$NAVVIS_WORKSPACE/3rdparty/libcvd:$NAVVIS_WORKSPACE/3rdparty/gvars3
  ```
- use the script ./launchROS_PTAM.sh to run PTAM with relocalization on a rosbag (see ./launchROS_PTAM.sh -h for all options)
- PTAM-GPL-ROS listens to ros topic 'camera/image_raw'


