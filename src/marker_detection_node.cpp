#include <ros/ros.h>
#include <marker_detection/marker_detection.h>
#include <iostream>
#include <vector>
using namespace std;

int main (int argc, char** argv)
{
	ros::init(argc, argv, "picture_joystick_node");
	ros::NodeHandle node_("~");

	// param settings
	string camera_topic_, header_frame_id_,child_frame_id_;
	int camera_frame_rate_;
	double tri_side_length_;
	bool reverse_, image_show_;
	node_.param("camera_topic", camera_topic_, string("/usb_cam/image_raw"));
	node_.param("camera_frame_rate", camera_frame_rate_, 30);
	node_.param("header_frame_id", header_frame_id_, string("camera"));
	node_.param("child_frame_id", child_frame_id_, string("marker"));
	node_.param("tri_side_length", tri_side_length_, 0.045);
	node_.param("image_show", image_show_, false);

	// initialize
	ros::Rate loop_rate(camera_frame_rate_);
	image_transport::ImageTransport it(node_);
	image_transport::Subscriber camera_sub = it.subscribe(camera_topic_, 1, imageCallback);
	image_transport::Publisher ImagePub = it.advertise("ResultImage", 1);
	ros::Publisher MarkerFlagPub = node_.advertise<std_msgs::Bool>("MarkerFlag",1);
	ros::Publisher MarkerPosePub = node_.advertise<geometry_msgs::PoseStamped>("MarkerPose",1);
	ros::Publisher MarkerPoseReversedPub = node_.advertise<geometry_msgs::PoseStamped>("MarkerPoseReversed",1);
	
	// hsv threshold init as blue
	HSV hsv_blue   = {112, 40, 30, 40};

	vector<Regiondata> regiondata;
	cv::Vec3d markerpose(0,0,0);
	bool MarkerFlag = false; 


	// start image windows
	if(image_show_) {
		cv::namedWindow("result");
		cv::namedWindow("binary");
		cv::startWindowThread();
	}

	while (ros::ok()) {
		//result_img = cv::imread("../monograph_fig/marker.png");
		if (ImgSub) {
			keychangeHSV(hsv_blue);   // QA WS ED RF TG -> Hue Range Sat Val List_number
			color_extract(result_img, binary_img, hsv_blue, 7);
			label(binary_img, regiondata, 100);
			MarkerFlag = markerpose_detection(result_img, binary_img, regiondata, markerpose, tri_side_length_);

			BroadcastTf(header_frame_id_, child_frame_id_, markerpose);
			PublishMarkerFlag(MarkerFlagPub, MarkerFlag);
			PublishPose(MarkerPosePub, markerpose, header_frame_id_);
			PublishPoseReversed(MarkerPoseReversedPub, markerpose, child_frame_id_);
			PublishResultImg(ImagePub, result_img);
			
			if(image_show_) {
				cv::imshow("result", result_img);
				cv::imshow("binary", binary_img);
				cv::waitKey(1);  // require for imshow
			}
		}
		ImgSub = false;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
