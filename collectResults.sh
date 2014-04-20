#!/bin/bash
#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

# ----------------------- determine which rosbag

#START_SEC="nan";
#SKIP_N_FRAMES="nan";
#FRAMES_BETWEEN_RELOC="nan";
#END_FRAME="nan"

#BAG=compressed_2012-01-25-15-24-45
BAG=compressed_2012-01-25-15-34-53
BAG=20120106_181812
BAG=20120106_181306

if [ $# -eq 1 ]; then 
  BAG=$1
elif [ $# -eq 2 ]; then
  BAG=$1
  SIGNATURE=$2
  #START_SEC=$2;
  #SKIP_N_FRAMES=$3;
  #FRAMES_BETWEEN_RELOC=$4;
  #END_FRAME=$5
fi

#SIGNATURE=$BAG'_Start'$START_SEC'_Skip'$SKIP_N_FRAMES'_Reloc'$FRAMES_BETWEEN_RELOC'_NFrames'$END_FRAME'_Processed_'$(date +%Y%m%d_%H%M%S);

# ----------------------- copy to signed data direcory
cd ../data
mkdir $SIGNATURE;
cd $SIGNATURE;
echo $PWD

cp /media/data/workspace_navvis/briefRelocalize/PTAM-GPL/log.txt .
mv ../frame_*.png .
mv ../keyPointMeasurements_*.txt .
mv ../posePlot* .
mv ../framePoses.txt .
mv ../keyFrameIDs.txt .
mv ../kFrame*.jpg .
mv ../kFpoints*.txt .
mv ../currFrame.jpg .
mv ../relocalizationFrameNr.txt .
mv ../imageRawHeaderReceivedInPTAM[A,U]*.txt .
mv ../kFrameBriefDescriptors*.txt .
mv ../frame2Seq.txt .
mv ../allKeyBriefDescriptors.txt .
mv ../allHistoryKeyBriefDescriptors.txt .
mv ../allKeyFramePoses.txt .
mv ../briefDescriptors.txt .
mv ../seqIdForSync.txt .
mv ../*.log .
ln -s /media/data/workspace_navvis/dataTablet/$BAG 'bagDataFolder'
#ln -s ../../dataTablet/$BAG/AccLog.txt
#ln -s ../../dataTablet/$BAG/GyroLog.txt 
#ln -s ../../dataTablet/$BAG/FrameInfo.txt 

# ----------------------- save current results in ./results
cd ../../visodo_briefRelocalize/results/
echo $PWD

rm *_dataSignature; 
rm *_bagDataFolder; 
rm newDataDir.txt;
#rm AccLog.txt GyroLog.txt FrameInfo.txt keyFrameIDs.txt framePoses.txt
#rm kFrame*.jpg kFpoints*.txt currFrame.jpg

#ln -s ../../data/$SIGNATURE/framePoses.txt
#ln -s ../../data/$SIGNATURE/keyFrameIDs.txt
#ln -s ../../data/$SIGNATURE/kFrame*.jpg
#ln -s ../../data/$SIGNATURE/kFpoints*.txt
#ln -s ../../data/$SIGNATURE/currFrame.jpg
ln -s ../../data/$SIGNATURE $SIGNATURE'_dataSignature'
ln -s ../../dataTablet/$BAG 'bagDataFolder'
#ln -s ../../dataTablet/$BAG/AccLog.txt
#ln -s ../../dataTablet/$BAG/GyroLog.txt 
#ln -s ../../dataTablet/$BAG/FrameInfo.txt

echo $PWD/$SIGNATURE'_dataSignature' > newDataDir.txt


