#!/bin/bash
#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

echo "--------------------------------------------------------------------------------"
echo " Launch programs necessary to process data with PTAM"
echo "--------------------------------------------------------------------------------"

if [ $# -eq 0 ];then
    echo "please specify at least a bag using the -b option"
    exit 1;
fi

# -------------------------- default parameters
BAG=20120106_181306.bag
START_SEC=0
SKIP_N_FRAMES=0
FRAMES_BETWEEN_RELOC=0
RELOC_TYPE=0
END_FRAME=9999
USE_MATLAB=0
WAIT_TIME_AFTER_RELOC=0
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
    -x|--skip)
      SKIP_N_FRAMES=$2; shift
      echo 'Skip '$SKIP_N_FRAMES' frames before one is handed over to PTAM'
      ;;
    -rt|--reloc-time)
      FRAMES_BETWEEN_RELOC=$2; shift
      echo 'Relocalize always after '$FRAMES_BETWEEN_RELOC' frames'
      ;;
    -rw|--reloc-wait-time)
      WAIT_TIME_AFTER_RELOC=$2; shift
      echo 'Wait for '$WAIT_TIME_AFTER_RELOC=' after relocalization'
      ;;
    -f|--feature)
      FEATURE_TYPE=$2; shift
      echo 'Use Feature Type '$FEATURE_TYPE
      ;;
   -r|--reloc-type)
      RELOC_TYPE=$2; shift
      echo 'Use relocaliser type '$RELOC_TYPE
      ;;
    -e|--end)
      END_FRAME=$2; shift
      echo 'Finish after '$END_FRAME' frames'     
      ;;
    #-m|--matlab)
    #  USE_MATLAB=1
    #  echo "using matlab to display statistics"
    #  ;;
    -h|--help)
      echo "valid configuration parameters are:"
      echo "-b |--bag <bagfile>:          .bag file to be played"
      echo "-s |--start <seconds>:        Start <seconds> into the dataset"
      echo "-x |--skip <frames>:          Skip <frames> frames before one is handed over to PTAM"
      echo "-rt|--reloc-time <frames>:    Relocalize always after <frames> frames"
      echo "-rw|--reloc-wait-time <dt>:   Wait for <dt> ms after relocalization before tracking is resumed"
      echo "-r |--reloc-type <typeNr>:    Use relocaliser of type <typeNr> (0=standard PTAM, "
      echo "                              1= BRIEF+LSH+PROSAC+MEstimator)"
      echo "-e |--end <frames>:           Stop after <frames> frames"
      echo "-f |--feature <frames>:       Feature type (0=BRIEF, 1=ORB, 2=FREAK)"
      echo "-h |--help:                   Display this help message"
      #echo "-m|--matlab:                use matlab to display statistics"
      exit 0;
      ;;
  esac
  shift       # Check next set of parameters.
done

if [ $BAG == "compressed_2012-01-25-15-24-45.bag" ]; then
  START_SEC=10; # geht gut bis zur Glastuere. (einmal Rotation laeuft gut!)  
  START_SEC=168; # started an der Treppe bis zur Glastuere
  START_SEC=210; # bad initialization
  START_SEC=10;
elif [ $BAG == "compressed_2012-01-25-15-34-53.bag" ]; then
  START_SEC=40; # Started bei Rotation am Glaskasten.
  START_SEC=78; # Started nachdem der Kerl vorbei ist. Geht gut bis zur Glastuer (vorherverliert er einmal noch, wegen glaskasten)
  START_SEC=78; 
elif [ $BAG == "20120106_181306.bag" ]; then
  START_SEC=3; # important for stability of solution!!!
elif [ $BAG == "20120223_110642.bag" ]; then
  START_SEC=1;
elif [ $BAG == "20120229_161021.bag" ]; then
  START_SEC=5;
elif [ $BAG == "20120229_161324.bag" ]; then
  START_SEC=2; # works somewhat - rotation destreus tracking
elif [ $BAG == "20120229_161427.bag" ]; then
  START_SEC=2; # doesnt work
elif [ $BAG == "20120302_173357.bag" ]; then
  START_SEC=6; # 6 works - 4 somwhat
elif [ $BAG == "gangHin.bag" ]; then
  START_SEC=6;
elif [ $BAG == "gangZurueck.bag" ]; then
  START_SEC=0;
else 
  echo 'no starting time in cache for rosbag! -> using '$START_SEC's';
fi

BAG_DIR=$(echo $BAG | sed s/.bag//);
SIGNATURE=$BAG_DIR'_Start'$START_SEC'_Skip'$SKIP_N_FRAMES'_FEAT'$FEATURE_TYPE'_Reloc'$FRAMES_BETWEEN_RELOC'_Type'$RELOC_TYPE'_RelocWait'$WAIT_TIME_AFTER_RELOC'_NFrames'$END_FRAME'_Processed_'$(date +%Y%m%d_%H%M%S);

echo "---------------------------------------------------------------------"
echo 'Signature: '$SIGNATURE
echo 'Start at '$START_SEC's'
echo 'Skip '$SKIP_N_FRAMES' frames before one is handed over to PTAM'
echo 'Relocalize using type '$RELOC_TYPE' always after '$FRAMES_BETWEEN_RELOC' frames'
echo 'Feature Type: '$FEATURE_TYPE' (0=BRIEF, 1=ORB, 2=FREAK)' 
echo 'Finish after '$END_FRAME' frames'
echo "---------------------------------------------------------------------"

# ------------------------ Check if roscore is running - if not the case start it
if [ -z $(ps axu | grep roscore | grep -v grep | grep -o roscore) ];
then
        echo 'roscore not running -> starting a new roscore'
        roscore &
	sleep 2
fi
# ------------------------ delete all previous data
rm ../data/frame_* 2> /dev/null
rm ../data/keyPoint* 2> /dev/null
rm ../data/posePlot* 2> /dev/null
rm ../data/framePoses.txt 2> /dev/null
rm ../data/keyFrameIDs.txt 2> /dev/null

# ------------------------ start programs 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/media/data/workspace_navvis/3rdparty/libcvd:/media/data/workspace_navvis/3rdparty/gvars3

killall konsole
konsole --noclose --new-tab --workdir $PWD/../rosbags -e rosbag play -s $START_SEC -d 6 $BAG #--pause $BAG -r 0.2
sleep 1;
konsole --noclose --new-tab --workdir $PWD -e ./imageDecompressionNode.sh 
sleep 1;
#cd PTAM-GPL
#gdb --args ./PTAM $SKIP_N_FRAMES $FRAMES_BETWEEN_RELOC $END_FRAME $RELOC_TYPE 
#konsole --noclose --new-tab --workdir $PWD -e ./startPTAM.sh $SKIP_N_FRAMES $FRAMES_BETWEEN_RELOC $END_FRAME $RELOC_TYPE $WAIT_TIME_AFTER_RELOC
konsole --noclose --new-tab --workdir $PWD -e bash -c "cd PTAM-GPL; ./PTAM $SKIP_N_FRAMES $FRAMES_BETWEEN_RELOC $END_FRAME $RELOC_TYPE $WAIT_TIME_AFTER_RELOC $FEATURE_TYPE 2>&1 | tee log.txt"
#just display the camera frames:
#rosrun image_view image_view image:=/camera/image_raw                                                                                                                                         
sleep 1

PTAM_PID=$(ps -ef | grep './[P]TAM' | awk '{ print $2 }')
echo PTAM PID: $PTAM_PID

PTAM_RUNNING=0; COUNTER=0;
echo -n "PTAM is running";
while [ $PTAM_RUNNING -eq 0 ]
do 
  sleep 1;
  $(kill -0 $PTAM_PID) 2> /dev/null;
  PTAM_RUNNING=$?;
  if [ $PTAM_RUNNING -eq 0 ]; then
    if [ $COUNTER -eq 3 ]; then
      echo -en "\b\b\b"
      COUNTER=0
    else
      echo -n "."
      COUNTER=`expr $COUNTER + 1`
    fi
  else
    echo "PTAM stopped running";
  fi
done

killall konsole

./collectResults.sh $BAG_DIR $SIGNATURE #$START_SEC $SKIP_N_FRAMES $FRAMES_BETWEEN_RELOC $END_FRAME
./zipForSebastian.sh $SIGNATURE
#if [ $USE_MATLAB -eq 1 ]; then
#  ./analyseWithMatlab.sh $BAG_DIR $END_FRAME $SIGNATURE
#fi
echo "All finished!"

stty echo # to fix issue with matlab
exit 0
