#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char** argv){

	// initialize
	ros::init(argc, argv, "position_correction");
	ros::NodeHandle node_("~");
	tf::TransformListener listener;

	int looprate_;
	double lookup_err_;
	string lookup_tf_header_, lookup_tf_child_;

	node_.param("looprate", looprate_, 30);
	node_.param("lookup_tf_header", lookup_tf_header_, string("/marker"));
	node_.param("lookup_tf_child", lookup_tf_child_, string("/cam"));

	ros::Rate looprate(looprate_);
	ros::Publisher movecmd_pub = node_.advertise<std_msgs::Float32MultiArray>("movecmd",1);

	double gain[3] = {15.0, 15.0, 1.5};
	double output[3] = {0,0,0};
	double rpy[3] = {0,0,0};
	double error[3] = {0,0,0};
	static tf::StampedTransform transform;

	sleep(3);
	while (node_.ok()){
		output[0] = 0;
		output[1] = 0;
		output[2] = 0;
		try{
			listener.lookupTransform(lookup_tf_header_, lookup_tf_child_, ros::Time(0), transform);
			transform.getBasis().getRPY(rpy[0], rpy[1], rpy[2]);
			error[0] = transform.getOrigin().x();
			error[1] = transform.getOrigin().y();
			error[2] = rpy[2];

			for(int i=0; i<3; i++) {
				output[i] = error[i] * gain[i];
				if(output[i]>1) output[i] = 1;
				if(output[i]<-1) output[i] = -1;
			}
		}	
		catch (tf::TransformException ex){
			static unsigned int errornum = 0;
			if (errornum%looprate_ == 0) ROS_ERROR("%s",ex.what());
			errornum++;
		}


		std_msgs::Float32MultiArray movecmd_msg;
		movecmd_msg.data.push_back(output[0]);
		movecmd_msg.data.push_back(output[1]);
		movecmd_msg.data.push_back(output[2]);
		movecmd_pub.publish(movecmd_msg);

		looprate.sleep();
	}
	return 0;
}
