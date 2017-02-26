#ifndef MARKER_DETECTION_H_
#define MARKER_DETECTION_H_


#include <ros/ros.h>
#include <string>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <kbhit.h>
#include <Labeling.h>
using namespace std;


cv::Mat result_img;
cv::Mat binary_img;
bool ImgSub = false;

struct Regiondata
{
	int pixels;
	cv::Point2d center;
	cv::Point size;
	cv::Point min;
	cv::Point max;
	unsigned char color;
	// compare areasize in sort function
	bool operator<(const Regiondata& another) const {
		return size.x * size.y < another.size.x * another.size.y;
	}
};

struct HSV
{
	int hue;
	int hue_range;
	int sat_min;
	int val_min;
};

void imageCallback(const sensor_msgs::ImageConstPtr&);

// change HSV list using keyboard "qawsedrftg"
void keychangeHSV(HSV&);

void color_extract(cv::Mat&, cv::Mat&, HSV, int blursize = 13);

// labeling (input mat is binary img)
void label(cv::Mat&, vector<Regiondata>& , unsigned int remove_pix = 200);

// if marker exist -> return true 
bool markerpose_detection(cv::Mat&, cv::Mat&, vector<Regiondata>, cv::Vec3d&, double);
bool samesize(Regiondata*, double threshold = 0.3);
bool triangle(Regiondata*, double threshold = 0.6);
// if 2 circle and 1 hollow circle --> return true
bool shape(Regiondata*, cv::Mat&);
void calc_markerpose(cv::Vec3d&, Regiondata*, cv::Mat&, double);

// draw arrow (Mat, bottom_point, top_point, color, thickness, arrowtop_size)
void arrow(cv::Mat&, cv::Point, cv::Point, cv::Scalar, int thickness = 2, int arrowtop_size = 10);

// transform opencv flame to center flame
// if inv = true  ->  center flame to opencv flame
cv::Vec2d tfcv2center(cv::Vec2d, cv::Vec2d, bool inv = false);

// rotate vector (double)[rad]
cv::Vec2d vec_rotation(cv::Vec2d, double);

// draw object position in image
void draw_text(cv::Vec3d, cv::Mat&);

void BroadcastTf(string, string, cv::Vec3d);
void PublishMarkerFlag(ros::Publisher&, bool);
void PublishPose(ros::Publisher&, cv::Vec3d, string);
void PublishPoseReversed(ros::Publisher&, cv::Vec3d, string);
void PublishResultImg(image_transport::Publisher&, cv::Mat&);

#endif
