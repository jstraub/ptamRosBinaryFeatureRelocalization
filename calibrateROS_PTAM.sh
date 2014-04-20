#!/bin/bash
#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

echo "-------------------------------------------------------"
echo "Launch everything necessary to process data with PTAM"
echo "-------------------------------------------------------"

if [ $# -eq 0 ];then
    echo "please specify at least a bag using the -b option"
    exit 1;
fi

# -------------------------- default parameters
BAG=20120201_145901_calib.bag
START_SEC=0
END_FRAME=9999
# ------------------------ determine run parameters for PTAM
while [ $# -gt 0 ]; do    # Until you run out of parameters . . .
  case "$1" in
    -b|--bag)
      BAG=$(echo $2 | sed 's/\///' | sed 's/.bag//')'.bag'; shift
      echo "Specified rosbag $BAG"
      PATH_TO_BAG="../rosbags/$BAG"
      if [ -f $PATH_TO_BAG ]; then
	    echo "  -> Found $PATH_TO_BAG"
      else
	    echo "  -> Did not find $PATH_TO_BAG"
    	exit 1
      fi      
      ;;
    -s|--start)
      START_SEC=$2; shift
      echo 'Start at '$START_SEC's of the dataset'
      ;;
    -h|--help)
      echo "valid config parameters are:"
      echo "-b|--bag <bagfile>:         .bag file to be played"
      echo "-s|--start <seconds>:       Start <seconds> into the dataset"
      echo "-h|--help:                  Display this help message"
      exit 0;
      ;;
  esac
  shift       # Check next set of parameters.
done

BAG_DIR=$(echo $BAG | sed s/.bag//);
SIGNATURE=$BAG_DIR'_Start'$START_SEC'_Processed_'$(date +%Y%m%d_%H%M%S);

echo "---------------------------------------------------------------------"
echo 'Signature: '$SIGNATURE
echo 'Start at '$START_SEC's'
echo "---------------------------------------------------------------------"

# ------------------------ Check if roscore is running - if not the case start it
if [ -z $(ps axu | grep roscore | grep -v grep | grep -o roscore) ];
then
        echo 'roscore not running -> starting a new roscore'
        roscore &
	sleep 2
fi
# ------------------------ start programs 
killall konsole
konsole --noclose --new-tab --workdir $PWD/../rosbags -e rosbag play -s $START_SEC -r 0.2 -d 6 $BAG #--pause $BAG -r 0.2
sleep 1;
konsole --noclose --new-tab --workdir $PWD -e ./imageDecompressionNode.sh 
sleep 1;
konsole --noclose --new-tab --workdir $PWD -e bash -c "cd PTAM; ./CameraCalibrator;"
#just display the camera frames:
#rosrun image_view image_view image:=/camera/image_raw                                                                                                                                         
sleep 1

#PTAM_PID=$(ps -ef | grep './[P]TAM' | awk '{ print $2 }')
#echo PTAM PID: $PTAM_PID

#PTAM_RUNNING=0; COUNTER=0;
#echo -n "PTAM is running";
#while [ $PTAM_RUNNING -eq 0 ]
#do 
#  sleep 1;
#  $(kill -0 $PTAM_PID) 2> /dev/null;
#  PTAM_RUNNING=$?;
#  if [ $PTAM_RUNNING -eq 0 ]; then
#    if [ $COUNTER -eq 3 ]; then
#      echo -en "\b\b\b"
#      COUNTER=0
#    else
#      echo -n "."
#      COUNTER=`expr $COUNTER + 1`
#    fi
#  else
#    echo "PTAM stopped running";
#  fi
#done
#
#killall konsole

exit 0
