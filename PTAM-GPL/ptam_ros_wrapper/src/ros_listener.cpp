#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include <stdio.h>
#include <ros/callback_queue.h>
#include "image_transport/image_transport.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <boost/bind.hpp>
#include <time.h>

// Additional includes for image conversion
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

using namespace cv;

typedef void (*callback_func_ptr)(CVD::Image<CVD::Rgb<CVD::byte> >, CVD::Image<CVD::byte>,
		vector<double>, vector<double>, vector<double>, int32_t, int32_t, int32_t);

typedef void (*img_callback_func_ptr)(CVD::Image<CVD::Rgb<CVD::byte> >, CVD::Image<CVD::byte>);

callback_func_ptr callbackFuncInPtam;
//img_callback_func_ptr imgCallbackFuncInPtam;


/**
 * Callback for IMU sensor messages
 */
//void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
//	ROS_INFO("Received a message");
//}

void imageCallback(const sensor_msgs::ImageConstPtr& cam_msg) {
  ROS_INFO("received image");

  std_msgs::Header header = cam_msg->header;
  ros::Time t = header.stamp;
  int32_t seq = header.seq;
//  printf("sec %d nano %d seq: %d\n", t.sec, t.nsec, seq);

  cv_bridge::CvImagePtr cv_ptr;
  try {
    // als BGR8 => bbbbbbbb gggggggg rrrrrrrr
    cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&) cv_ptr->image;
  CVD::ImageRef imref = CVD::ImageRef(cv_ptr->image.size().width,
      cv_ptr->image.size().height);

  // CVD Umwandlungsformeln, siehe http://ewokrampage.wordpress.com/video-sources/
//  printf("creating RGB img\n");
  // RGB Bild
  CVD::Image<CVD::Rgb<CVD::byte> > imRGB;
  imRGB.resize(imref);
  for (int i = 0; i < cv_ptr->image.size().height; i++) {
    for (int j = 0; j < cv_ptr->image.size().width; j++) {
      imRGB[i][j].red = frame_p(i, j)[2];
      imRGB[i][j].green = frame_p(i, j)[1];
      imRGB[i][j].blue = frame_p(i, j)[0];
    }
  }
//  printf("creating SW img\n");
  // SW Bild
  CVD::Image<CVD::byte> imBW;
  imBW.resize(imref);
  for (int i = 0; i < cv_ptr->image.size().height; i++) {
    for (int j = 0; j < cv_ptr->image.size().width; j++) {
      imBW[i][j] = (frame_p(i, j)[0] + frame_p(i, j)[1]
                                                     + frame_p(i, j)[2]) / 3;
    }
  }

  vector<double> a(4,0.0);
  vector<double> b(3,0.0);
  vector<double> c(3,0.0);

//  printf("calling PTAM callback\n");
  callbackFuncInPtam(imRGB, imBW, a, b, c, t.sec, t.nsec, seq);
}

//void rosCallback(const sensor_msgs::ImageConstPtr& cam_msg,
//		const sensor_msgs::ImuConstPtr& imu_msg) {
//	//ROS_INFO("received image and imu data");
//
//	//////////////////////////////////
//	// **** Timestamp ****
//	//////////////////////////////////
//	std_msgs::Header header = imu_msg->header;
//	ros::Time t = header.stamp;
//	int32_t seq = header.seq;
//
//	// use the timestamp of the image
//	header = cam_msg->header;
//	t = header.stamp;
//	seq = header.seq;
//	printf("sec %d nano %d seq: %d\n", t.sec, t.nsec, seq);
//
//	//////////////////////////////////
//	// **** IMU Data ****
//	//////////////////////////////////
//	geometry_msgs::Quaternion orientation = imu_msg->orientation;
//	vector<double> vOrientation;
//	vOrientation.push_back(orientation.x);
//	vOrientation.push_back(orientation.y);
//	vOrientation.push_back(orientation.z);
//	vOrientation.push_back(orientation.w);
//
//	geometry_msgs::Vector3 angular_velocity = imu_msg->angular_velocity;
//	vector<double> vVelocity;
//	vVelocity.push_back(angular_velocity.x);
//	vVelocity.push_back(angular_velocity.y);
//	vVelocity.push_back(angular_velocity.z);
//
//	geometry_msgs::Vector3 linear_acceleration = imu_msg->linear_acceleration;
//	vector<double> vAcceleration;
//	vAcceleration.push_back(linear_acceleration.x);
//	vAcceleration.push_back(linear_acceleration.y);
//	vAcceleration.push_back(linear_acceleration.z);
//
//	//////////////////////////////////
//	// **** Image Data ****
//	//////////////////////////////////
//
//	cv_bridge::CvImagePtr cv_ptr;
//	try {
//		// als BGR8 => bbbbbbbb gggggggg rrrrrrrr
//		cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
//	} catch (cv_bridge::Exception& e) {
//		ROS_ERROR("cv_bridge exception: %s", e.what());
//		return;
//	}
//
//	Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&) cv_ptr->image;
//	CVD::ImageRef imref = CVD::ImageRef(cv_ptr->image.size().width,
//			cv_ptr->image.size().height);
//
//	// CVD Umwandlungsformeln, siehe http://ewokrampage.wordpress.com/video-sources/
//
//	// RGB Bild
//	CVD::Image<CVD::Rgb<CVD::byte> > imRGB;
//	imRGB.resize(imref);
//	for (int i = 0; i < cv_ptr->image.size().height; i++) {
//		for (int j = 0; j < cv_ptr->image.size().width; j++) {
//			imRGB[i][j].red = frame_p(i, j)[2];
//			imRGB[i][j].green = frame_p(i, j)[1];
//			imRGB[i][j].blue = frame_p(i, j)[0];
//		}
//	}
//
//	// SW Bild
//	CVD::Image<CVD::byte> imBW;
//	imBW.resize(imref);
//	for (int i = 0; i < cv_ptr->image.size().height; i++) {
//		for (int j = 0; j < cv_ptr->image.size().width; j++) {
//			imBW[i][j] = (frame_p(i, j)[0] + frame_p(i, j)[1]
//					+ frame_p(i, j)[2]) / 3;
//		}
//	}
//
//	//////////////////////////////////
//	// send the received data to the callback function
//	//////////////////////////////////
//	callbackFuncInPtam(imRGB, imBW, vOrientation, vVelocity, vAcceleration, t.sec, t.nsec, seq);
//}

///////////////////////////////////////////////////////
// Subscriber class
///////////////////////////////////////////////////////

class SubscriberClass {
private:

//	message_filters::Subscriber<sensor_msgs::Image>* camSubscriber;
//	message_filters::Subscriber<sensor_msgs::Imu>* imuSubscriber;

	image_transport::ImageTransport * it;
	image_transport::Subscriber imgSubscriber;

	// Sync policy for approximate time stamps to synchronize image and imu events
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
//			sensor_msgs::Imu> mySyncPolicy;
//	// Synchronizer using this policy
//	message_filters::Synchronizer<mySyncPolicy>* imuCamSynchronizer;

public:
	SubscriberClass() {
		// need to declare argc and argv as variables instead of directly passing 0 and NULL in order to
		// make the signature of the function clear
		int argc = 0;
		char ** argv = NULL;
		ros::init(argc, argv, "ptam_listener");
		printf("ros_init called\n");

		//cv::namedWindow(WINDOW);
	}

	void subscribeToImuAndCam(void(*callback)(CVD::Image<CVD::Rgb<CVD::byte> >, CVD::Image<CVD::byte>,
	    vector<double>, vector<double>, vector<double>, int32_t, int32_t, int32_t)) {
	  printf("subscribeToImuAndCam called\n");

//	  ros::NodeHandle nodeHandle;
//	  camSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(
//	      nodeHandle, "/camera/image_raw", 20);
//	  imuSubscriber = new message_filters::Subscriber<sensor_msgs::Imu>(
//	      nodeHandle, "/imu/data", 20);
//	  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//	  imuCamSynchronizer = new message_filters::Synchronizer<mySyncPolicy>(
//	      mySyncPolicy(100), *camSubscriber, *imuSubscriber);
//	  imuCamSynchronizer->registerCallback(boost::bind(&rosCallback, _1, _2));

	  callbackFuncInPtam = callback;
	  // just for testing, subscribe to image alone
	  ros::NodeHandle nh;
	  it = new image_transport::ImageTransport(nh);
	  imgSubscriber = it->subscribe("camera/image_raw", 0, imageCallback); // queue size 0 -> is inifinite
	  // end testing
	}
};

///////////////////////////////////////////////////////
// Functions that are used by PTAM
///////////////////////////////////////////////////////

SubscriberClass *subscriber;

/**
 * Calls the ros::spinOnce method, which causes the callbacks to be called
 */
extern "C" void ros_spinOnce() {
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0)); // was 0.1
}

/**
 * Calls ros::init(), with argc und argv set to 0 and null
 */
extern "C" void ros_init() {
	subscriber = new SubscriberClass();
}

/**
 * Subscribes to the IMU and the camera, passing the callback function as argument
 */
extern "C" void ros_subscribe(void(*callback)(CVD::Image<CVD::Rgb<CVD::byte> >, CVD::Image<CVD::byte>,
		vector<double>, vector<double>, vector<double>, int32_t, int32_t, int32_t)) {
	subscriber->subscribeToImuAndCam(callback);
}
