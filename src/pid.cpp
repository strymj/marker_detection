#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char** argv){

	// initialize
	ros::init(argc, argv, "pid");
	ros::NodeHandle node_("~");
	tf::TransformListener listener;

	int looprate_;
	double gain_p_, gain_i_, gain_d_;
	double master_gain_x_, master_gain_y_, master_gain_yaw_;
	string lookup_tf_header_, lookup_tf_child_;

	node_.param("looprate", looprate_, 30);
	node_.param("lookup_tf_header", lookup_tf_header_, string("/input_cam"));
	node_.param("lookup_tf_child", lookup_tf_child_, string("/vehicle_cam"));
	node_.param("gein_d", gain_p_, 0.2);
	node_.param("gein_i", gain_i_, 3.0);
	node_.param("gain_d", gain_d_, 0.004);
	node_.param("master_gain_x", master_gain_x_, 1.0);
	node_.param("master_gain_y", master_gain_y_, 1.0);
	node_.param("master_gain_yaw", master_gain_yaw_, 1.0);

	ros::Rate looprate(looprate_);
	ros::Publisher movecmd_pub = node_.advertise<std_msgs::Float32MultiArray>("movecmd",1);
	ros::Publisher x_pub = node_.advertise<std_msgs::Int32>("x",1);
	ros::Publisher y_pub = node_.advertise<std_msgs::Int32>("y",1);
	ros::Publisher t_pub = node_.advertise<std_msgs::Int32>("t",1);

	double roll_prop, pitch_prop;
	double error_p[3] = {0,0,0};
	double error_i[3] = {0,0,0};
	double error_d[3] = {0,0,0};
	double eminus1[3] = {0,0,0};
	double output[3] = {0,0,0};
	double master_gain_x=1, master_gain_y=1, master_gain_yaw=1;

	int tflookup = 0;

	while (node_.ok()){

		// listen tf	
		static tf::StampedTransform transform;
		try{
			listener.lookupTransform("/vehicle_cam", "/input_cam", ros::Time(0), transform);

			error_p[0] = transform.getOrigin().x();
			error_p[1] = transform.getOrigin().y();
			transform.getBasis().getRPY(roll_prop, pitch_prop, error_p[2]);

			error_i[0] = error_p[0] - eminus1[0];
			error_i[1] = error_p[1] - eminus1[1];
			error_i[2] = error_p[2] - eminus1[2];

			error_d[0] += error_p[0];
			error_d[1] += error_p[1];
			error_d[2] += error_p[2];

			eminus1[0] = error_p[0];
			eminus1[1] = error_p[1];
			eminus1[2] = error_p[2];

			tflookup = 0;
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			tflookup += 1;
			sleep(3);
		}

		if(tflookup==0) {
			output[0]   = - gain_p_ * error_p[0] - gain_i_ * error_d[0] - gain_d_ * error_d[0];
			output[1]   = - gain_p_ * error_p[1] - gain_i_ * error_d[1] - gain_d_ * error_d[1];
			output[2]   = - gain_p_ * error_p[2] - gain_i_ * error_d[2] - gain_d_ * error_d[2];

			output[0] *= master_gain_x;
			output[1] *= master_gain_y;
			output[2] *= master_gain_yaw;
		}
		else if(5<=tflookup) {
			output[0] = 0.0;
			output[1] = 0.0;
			output[2] = 0.0;
		}
		std_msgs::Int32 x_msg;
		std_msgs::Int32 y_msg;
		std_msgs::Int32 t_msg;
		x_msg.data = 1234;
		y_msg.data = output[1] * 1000;
		t_msg.data = output[2] * 1000;
		x_pub.publish(x_msg);
		y_pub.publish(y_msg);
		t_pub.publish(t_msg);

		std_msgs::Float32MultiArray movecmd_msg;
		movecmd_msg.data.push_back(output[0]);
		movecmd_msg.data.push_back(output[1]);
		movecmd_msg.data.push_back(output[2]);
		movecmd_pub.publish(movecmd_msg);

		looprate.sleep();
	}
	return 0;
}
