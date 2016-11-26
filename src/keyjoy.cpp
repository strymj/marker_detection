#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "kbhit.h"
using namespace std;


void keyjoy(double* movecmd)
{
	static double x=0, y=0, t=0, ratio = 1.0;
	enum {UP='u', DOWN='j', RIGHT='k', LEFT='h', ROTATE='y', COUNTER='i', RATIO_U='r', RATIO_D='f'};
	if (kbhit()) {
		short int key;
		key = getchar();
		switch(key){
			case UP: 
				x= 0; y= 1; t= 0; break;
			case DOWN:
				x= 0; y=-1; t= 0; break;
			case RIGHT:
				x= 1; y= 0; t= 0; break;
			case LEFT:
				x=-1; y= 0; t= 0; break;
			case ROTATE:
				x= 0; y= 0; t= 1; break;
			case COUNTER:
				x= 0; y= 0; t=-1; break;
			default:
				x= 0; y= 0; t= 0; break;
			case RATIO_U:
				ratio += 0.1;
				if(ratio>1.0) ratio = 1.0;
				cout<<"   ratio : "<<ratio<<endl;
				break;
			case RATIO_D:
				ratio -= 0.1;
				if(ratio<0.0) ratio = 0.0;
				cout<<"   ratio : "<<ratio<<endl;
				break;
		}
		movecmd[0] = x * ratio;
		movecmd[1] = y * ratio;
		movecmd[2] = t * ratio;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyjoy");
	ros::NodeHandle node_("~");

	int looprate_;
	node_.param("looprate", looprate_, 10);

	ros::Rate looprate(looprate_);
	ros::Publisher movecmd_pub = node_.advertise<std_msgs::Float32MultiArray>("movecmd", 1);

	double movecmd[3] = {0,0,0};

	while(ros::ok()) {
		keyjoy(movecmd);
		std_msgs::Float32MultiArray movecmd_msg;
		movecmd_msg.data.push_back(movecmd[0]);
		movecmd_msg.data.push_back(movecmd[1]);
		movecmd_msg.data.push_back(movecmd[2]);
		movecmd_pub.publish(movecmd_msg);
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
