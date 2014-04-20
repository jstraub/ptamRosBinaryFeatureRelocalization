# DO NOT DELETE THIS LINE -- make depend depends on it.


# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++
COMPILEFLAGS = -I ../gvars3 -I ..  -I ../libcvd -I /usr/local/include -I../../localizeDB/include -D_LINUX -D_REENTRANT -Wall -march=nocona -msse3 `pkg-config --cflags glib-2.0` `pkg-config --cflags gstreamer-0.10` -g -O2

#LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lGVars3 -lcvd
LINKFLAGS = -L /usr/local/lib -lGVars3 -lcvd -lpng `pkg-config --libs glib-2.0` `pkg-config --libs gstreamer-app-0.10` -L /usr/lib -llapack -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_highgui
#LINKFLAGS = -L ../gvars3 -Wl,-rpath=$../gvars3  -lGVars3 -L ../libcvd -Wl,-rpath=$../libcvd -lcvd -llapack `pkg-config --libs glib-2.0` `pkg-config --libs gstreamer-app-0.10`

# Edit this line to change video source
#VIDEOSOURCE = VideoSource_Linux_DV.o
#VIDEOSOURCE = VideoSource_Linux_Gstreamer_File.o
VIDEOSOURCE = VideoSource_Linux_ROS.o

OBJECTS=	main.o\
		GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		System.o \
		ATANCamera.o\
		KeyFrame.o\
		MapPoint.o\
		Map.o\
		SmallBlurryImage.o\
		ShiTomasi.o \
		HomographyInit.o \
		MapMaker.o \
		Bundle.o \
		PatchFinder.o\
		Relocaliser.o\
		MiniPatch.o\
		MapViewer.o\
		ARDriver.o\
		EyeGame.o\
		Tracker.o\
		BriefRelocalizer.o\
		BriefExtractor.o\
		BriefData.o\
		outputHelper.o\
		Classifier.o\
		Ransac.o
		
CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o

all: PTAM CameraCalibrator

PTAM: $(OBJECTS)
	$(CC) -o PTAM $(OBJECTS) $(LINKFLAGS)

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -o CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS)

ros:
	-ROS_PACKAGE_PATH=$$ROS_PACKAGE_PATH:$$PWD/ptam_ros_wrapper rosmake ptam_ros_wrapper
	-cp ptam_ros_wrapper/lib/libptam_ros_wrapper.so lib

%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm *.o

clean-ros:
	-$(MAKE) -C ptam_ros_wrapper clean

depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.h


-include dependencies









