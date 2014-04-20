/*
 * ROSImporter.cc
 *
 *  Created on: Sep 2, 2011
 *      Author: ga69qel
 *
 *  This class serves as wrapper for the ROS functions provided by the ptam_ros_wrapper library.
 *  It subscribes to ROS topics that send IMU and camera data and receives the messages
 *  by a callback function.
 */
#include "ROSImporter.h"
#include <stdio.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

typedef int (*testFunc_ptr)();
typedef int (*testFunc2_ptr)(int);

// Typedefs for the function pointers in order to cast them from the dlsym call
typedef void (*RosInit_ptr)();
typedef void (*RosSpinOnce_ptr)();
typedef void (*RosImuSubscribe_ptr)(void (*callback)(double, double, double));
typedef void (*RosSubscribe_ptr)(void (*callback)());

// Variables for the functions to be called
RosInit_ptr rosInit;
RosSpinOnce_ptr rosSpinOnce;
RosImuSubscribe_ptr rosImuSubscribe;
RosSubscribe_ptr rosSubscribe;

void* rostest_handle;
int (*fn)();
char* error;


void ROSImporter::ImportROSFunctions() {
	// Open the library
	void * handle;
	handle = dlopen("lib/libptam_ros_wrapper.so", RTLD_LAZY);
	if (!handle) {
		fputs(dlerror(), stderr);
		exit(1);
	}

	testFunc_ptr testFunc;

	testFunc = (testFunc_ptr) dlsym(handle, "testFunc");
	if ((error = dlerror()) != NULL) {
		fputs(error, stderr);
		exit(1);
	}

	// Get the function handles
	rosInit = (RosInit_ptr) dlsym(handle, "ros_init");
	if ((error = dlerror()) != NULL) {
		fputs(error, stderr);
		exit(1);
	}

	rosSpinOnce = (RosSpinOnce_ptr) dlsym(handle, "ros_spinOnce");
	if ((error = dlerror()) != NULL) {
		fputs(error, stderr);
		exit(1);
	}

	rosImuSubscribe = (RosImuSubscribe_ptr) dlsym(handle, "ros_imuSubscribe");
	if ((error = dlerror()) != NULL) {
		fputs(error, stderr);
		exit(1);
	}

	rosSubscribe = (RosSubscribe_ptr) dlsym(handle, "ros_subscribe");
	if ((error = dlerror()) != NULL) {
		fputs(error, stderr);
		exit(1);
	}

	//int i = testFunc2(3);
	//printf("Value from Library: %i", testFunc());
	//testFunc();
	RosInit();
	//RosSubscribeToImu();
	RosSubscribe();
	for (int i=0; i<20; i++) {
		RosSpinOnce();
	}
	dlclose(handle);
}

/**
 * Initializes the ROS package. Must be called first.
 */
void ROSImporter::RosInit() {
	rosInit();
}

/**
 * Must be called in the loop to trigger the callback
 */
void ROSImporter::RosSpinOnce() {
	rosSpinOnce();
}

/**
 * Subscribe to the /imu/data topic. Currently, the topic is fixed
 */
void ROSImporter::RosSubscribeToImu() {
	rosImuSubscribe(ImuCallback);
}

void ROSImporter::RosSubscribe() {
	rosSubscribe(RosCallback);
}

void ROSImporter::ImuCallback(double x, double y, double z) {
	printf("I heard: [%f %f %f]\n", x,y,z);
}

void ROSImporter::RosCallback() {
	printf("RosCallback");
}
