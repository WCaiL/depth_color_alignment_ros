#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "depth2colorAlign.h"
#include <ctime>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;


void callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth, ros::Publisher pub)
{
	cv_bridge::CvImagePtr cv_ptr_rgb;
	cv_bridge::CvImagePtr cv_ptr_depth;
	try
	{
		cv_ptr_rgb = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
		cv_ptr_depth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat im_color = cv_ptr_rgb->image;
	cv::Mat im_depth = cv_ptr_depth->image;

	im_color.convertTo(im_color, CV_16UC3);
	im_color = im_color * 255;

	// cv::namedWindow("color", CV_RAND_NORMAL);
	// imshow("color", im_color);
	// cv::namedWindow("depth", CV_RAND_NORMAL);
	// imshow("depth", 4 * im_depth);
	// cv::waitKey();

	cv::Mat im_registrated_depth;

	cv::Mat Transform = (cv::Mat_<double>(4, 4)
		<< 1, 0, 0, -0.0777703,
		   0, 1, 0, 0,
		   0, 0, 1, 0,
		   0, 0, 0, 1);

	cv::Mat K_color = (cv::Mat_<double>(3, 3)
		<< 570.342204, 0, 319.5,
		   0, 570.342204, 239.5,
		   0, 0, 1);

	cv::Mat K_depth = (cv::Mat_<double>(3, 3)
		<< 570.342204, 0, 319.5,
		   0, 570.342204, 239.5,
		   0, 0, 1);

	cv::Size color_size(im_color.cols, im_color.rows);
	cv::Size depth_size(im_depth.cols, im_depth.rows);

	Depth2ColorAlign app(color_size, depth_size, Transform, K_color, K_depth);


	app.align(im_depth, im_registrated_depth);


	cv_bridge::CvImage out_msg;
	out_msg.header = depth->header;
	out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	out_msg.image = im_registrated_depth;
	pub.publish(out_msg.toImageMsg());

	cout << "Published: " << out_msg.header.stamp << endl;

	// imshow("registrated depth", im_registrated_depth * 8);
}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "depth_color_alignment");
    ros::start();


	ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/camera/depth_registered/image_raw", 100);
    sync.registerCallback(boost::bind(&callback,_1,_2, pub));

	cout << "Waiting for images..." << endl;

	ros::spin();


	return 0;
}
