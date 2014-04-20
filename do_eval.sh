#!/bin/bash
#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

cd test/build/
pwd
#DT='0 25 50 75 100 125 150 175 200 225 250 275 300 325 350 375 400 425 450 475 500 525 550 575 600 625 650 675 700 725 750 775 800' # in frames
DT='900 1000 1100 1200 1300 1400' # 1500 2000'
FEATS='0 3' # BRIEF=0 ORB=1 FREAK=2 usBRIEF=3
HISTS='1 0' # PROSAC full history (1) and only updates (0)
for HIST in $HISTS
do
  for FEAT in $FEATS
  do
    for DT_I in $DT
    do
      echo "./testLSHRansacLoc -m 14 -l 11 -t 3 -q 10 -r $RELOC_WAIT;"
      ./testLSHRansacLoc -m 14 -l 11 -t 3 -q 5 -p 1 -u $HIST -f $FEAT -r 100 -d $DT_I &
    done
  done
done
cd ../../
pwd
exit;



cd test/build/
pwd
WAITS='30 100 300' 
for RELOC_WAIT in $WAITS
do
  echo "./testLSHRansacLoc -m 14 -l 11 -t 3 -q 10 -r $RELOC_WAIT;"
#  ./testLSHRansacLoc -m 14 -l 11 -t 3 -q 5 -p 0 -r $RELOC_WAIT; # RANSAC
  ./testLSHRansacLoc -m 14 -l 11 -t 3 -q 5 -p 1 -f 1 -r $RELOC_WAIT; # PROSAC full history
done
for RELOC_WAIT in $WAITS
do
  echo "./testLSHRansacLoc -m 14 -l 11 -t 3 -q 10 -r $RELOC_WAIT;"
  ./testLSHRansacLoc -m 14 -l 11 -t 3 -q 5 -p 1 -f 0 -r $RELOC_WAIT; # PROSAC always update feature observation
done
cd ../../
pwd
exit;


i=0; I=5;
while [ $i -lt $I ]
do
  echo './launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800;'
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 0 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 1 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 2 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 3 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 10 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 11 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 12 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 13 -rt 4800;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 0 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 1 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 2 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 3 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 10 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 11 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 12 -rt 1500 -e 2998;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -f 13 -rt 1500 -e 2998;
  i=$(echo "$i + 1" | bc -l)
done
exit;



i=0; I=9;
while [ $i -lt $I ]
do
  echo './launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800;'
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800;
  i=$(echo "$i + 1" | bc -l)
done

exit;

i=0; I=4;
while [ $i -lt $I ]
do
  echo './launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800;'
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1; #-rt 4800;
  i=$(echo "$i + 1" | bc -l)
done


#WAITS='100 200 300 400 500 700 900 1200 1500 2000 3000 5000 10000 30000'
#WAITS='100 1200 10000 30000'
#WAITS='100 300 700 1000 1200 1500 2000 3000'
WAITS='5000 10000 15000'
for RELOC_WAIT in $WAITS
do
  echo "./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800 -rw $RELOC_WAIT;"
  echo "./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 1500 -e 2998 -rw $RELOC_WAIT;"
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800 -rw $RELOC_WAIT;
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 1500 -e 2998 -rw $RELOC_WAIT;
done


exit;

i=0; I=10;
while [ $i -lt $I ]
do
  echo './launchROS_PTAM.sh -b 20120420_175544.bag -s 1 -r 1 -rt 1500 -e 2998;'
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 1500 -e 2998;
  i=$(echo "$i + 1" | bc -l)
done

i=0; I=10;
while [ $i -lt $I ]
do
  echo './launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 0 -rt 1500 -e 2998;'
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 0 -rt 1500 -e 2998;
  i=$(echo "$i + 1" | bc -l)
done


i=0; I=10;
while [ $i -lt $I ]
do
  echo './launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800;'
  ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 1 -rt 4800;
  i=$(echo "$i + 1" | bc -l)
done




exit; 







#./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 1500 -e 3500; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 1500 -e 3500; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 1500 -e 3500; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 4800; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 4800; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 4800; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 0 -rt 4800; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 0 -rt 4800; ./launchROS_PTAM.sh -b 20120420_175544.bag -s 2 -r 2 -rt 4800; 

