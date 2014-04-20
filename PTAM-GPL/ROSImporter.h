/*
 * ROSImporter.h
 *
 *  Created on: Sep 2, 2011
 *      Author: ga69qel
 */

#ifndef ROSIMPORTER_H_
#define ROSIMPORTER_H_

#include <stdio.h>
#include <dlfcn.h>
#include <stdlib.h>

class ROSImporter
{
public:
	static void ImportROSFunctions();
	static void RosInit();
	static void RosSpinOnce();
	static void RosSubscribeToImu();
	static void RosSubscribe();
	static void RosCallback();
	static void ImuCallback(double x, double y, double z);
};


#endif /* ROSIMPORTER_H_ */
