#include <marker_detection/marker_detection.h>
using namespace std;
using namespace cv;

void keychangeHSV(HSV& list)
{
	static int n = 0;
	enum {H_UP='q', H_DN='a', R_UP='w', R_DN='s', S_UP='e', S_DN='d', V_UP='r', V_DN='f'};
	if (kbhit()) {
		short int key;
		key = getchar();
		switch(key){
			case H_UP: list.hue = ++list.hue%180; break;
			case H_DN: list.hue = --list.hue%180; break;
			case R_UP: ++list.hue_range; break;
			case R_DN: --list.hue_range; break;
			case S_UP: ++list.sat_min; break;
			case S_DN: --list.sat_min; break;
			case V_UP: ++list.val_min; break;
			case V_DN: --list.val_min; break;
			default:;
		}
		cout<<" hue:"<<list.hue<<" range:"<<list.hue_range<<" sat_min:"<<list.sat_min<<" val_min:"<<list.val_min<<endl;
	}
}

void color_extract(Mat& input_img, Mat& output_img, HSV list, int blursize)
{	
	int hue_min = (list.hue - list.hue_range/2)%180;
	int hue_max = (list.hue + list.hue_range/2)%180;
	output_img = Mat::zeros(Size(input_img.cols, input_img.rows), CV_8UC1);
	cv::Mat comp_img;
	cvtColor(input_img, comp_img, CV_BGR2HSV);
	for(int y=0; y<comp_img.rows; y++){
		Vec3b *src_i = comp_img.ptr<Vec3b>(y);
		for(int x=0; x<comp_img.cols; x++){
			if(hue_min<=hue_max) {
				if(list.sat_min < src_i[x][1]
					&& list.val_min < src_i[x][2] 
					&& hue_min< src_i[x][0] && src_i[x][0] < hue_max) {
				output_img.at<unsigned char>(y,x) = 255;
				}
			}
			else {
				if(list.sat_min < src_i[x][1]
					&& list.val_min < src_i[x][2] 
					&& (src_i[x][0] < hue_max || hue_min< src_i[x][0])) {
				output_img.at<unsigned char>(y,x) = 255;
				}
			}
		}
	}
	medianBlur(output_img, output_img, blursize);
}

void label(Mat& binary_img, vector<Regiondata>& regiondata, unsigned int remove_pix)
{
	regiondata.clear();
	Mat label(binary_img.size(), CV_16SC1);
	LabelingBS labeling;
	labeling.Exec(binary_img.data, (short *)label.data, binary_img.cols, binary_img.rows, true, remove_pix);	
	RegionInfoBS *ri;
	for( int i = 0; i < labeling.GetNumOfResultRegions(); i++)
	{
		Regiondata now;
		ri = labeling.GetResultRegionInfo(i);
		//Mat labelarea;
		//compare(label, i+1, labelarea, CV_CMP_EQ);
		// fighting!!! minEnclosingCircle(labelarea, data[i].c_center, data[i].radius);
		//Mat color(binary_img.size(), CV_8UC3, Scalar(255,255,255));
		//color.copyTo(label_img, labelarea);
		now.pixels = ri->GetNumOfPixels(); 
		Point2f center;
		ri->GetCenter(center.x, center.y); 
		now.center.x = center.x;
		now.center.y = center.y;
		ri->GetSize(now.size.x, now.size.y);
		ri->GetMin(now.min.x, now.min.y);
		ri->GetMax(now.max.x, now.max.y);
		now.color = ri->GetSourceValue();
		regiondata.push_back(now);
	}
	//sort(regiondata.begin(), regiondata.end());
}

bool markerpose_detection(Mat& image, Mat& binimg,vector<Regiondata> regiondata, Vec3d& markerpose, double sidelen)
{
	bool marker_exist = false;
	vector<Vec2d> marker_circle;
	int num1=-1, num2=-1, num3=-1;
	double normmin = -1.0;
	Regiondata comp[3];
	for(int i=0; i<regiondata.size(); i++) {
		for(int j=i+1; j<regiondata.size(); j++) {
			for(int k=j+1; k<regiondata.size(); k++) {
				comp[0] = regiondata[i];
				comp[1] = regiondata[j];
				comp[2] = regiondata[k];
				if(samesize(comp) && triangle(comp) && shape(comp,binimg)) {
					double normsum = 0.0;
					for(int l=0; l<3; l++) {
						Vec2d vec = comp[l].center - comp[(l+1)%3].center;
						normsum += norm(vec);
					}
					if(normsum < normmin || normmin < 0) {
						normmin = normsum;
						num1 = i;
						num2 = j;
						num3 = k;
					}
				}
			}
		}
	}
	if(num1 != -1 && num2 != -1 && num3 != -1) {
		comp[0] = regiondata[num1];
		comp[1] = regiondata[num2];
		comp[2] = regiondata[num3];
		calc_markerpose(markerpose, comp, image, sidelen);
		marker_exist = true;
	}
	draw_text(markerpose, image);
	return marker_exist;
}

bool samesize(Regiondata* region, double threshold)
{
	bool ans = true;
	for(int i=0; i<3; i++) {
		int x1 = region[i].size.x;
		int x2 = region[(i+1)%3].size.x;
		int y1 = region[i].size.y;
		int y2 = region[(i+1)%3].size.y;
		if( fabs(1-atan2(x1,x2)) > threshold ) {
			ans = false;
		}
		if( fabs(1-atan2(y1,y2)) > threshold ) {
			ans = false;
		}
	}
	return ans;
}

bool triangle(Regiondata* region, double threshold)
{
	bool ans = true;
	for(int i=0; i<3; i++) {
		double norm_v1 = norm(Vec2d(
					region[i].center.x-region[(i+1)%3].center.x,
					region[i].center.y-region[(i+1)%3].center.y));
		double norm_v2 = norm(Vec2d(
					region[(i+1)%3].center.x-region[(i+2)%3].center.x,
					region[(i+1)%3].center.y-region[(i+2)%3].center.y));
		if ( fabs(1-atan2(norm_v1, norm_v2)) > threshold ) {
			ans = false;
			break;
		}
	}
	return ans;
}

bool shape(Regiondata* region, Mat& binimg)
{
	bool ans = false;
	int white = 0;
	int blue = 0;
	for(int i=0; i<3; i++) {
		unsigned char centercolor = binimg.at<unsigned char>(region[i].center.y, region[i].center.x);
		if(centercolor == 0) white++;
		if(centercolor == 255) blue++;
	}
	if(white==1 && blue==2) {
		ans = true;
		//for(int i=0; i<3; i++) {
		//	Vec2d circle(region[(i+green)%3].center.x, region[(i+green)%3].center.y);
		//	circles.push_back(circle);
		//}
		//if(circles[1][0]*circles[2][1]-circles[1][1]*circles[2][0] > 0) {
		//	swap(circles[1], circles[2]);
		//}
	}
	return ans;
}

void calc_markerpose(Vec3d& object_pose, Regiondata* ccl, Mat& image, double sidelen)
{
	// define T,L,R number
	double ratio[3];
	ratio[0] = (double)ccl[0].pixels / (ccl[0].size.x * ccl[0].size.y);
	ratio[1] = (double)ccl[1].pixels / (ccl[1].size.x * ccl[1].size.y);
	ratio[2] = (double)ccl[2].pixels / (ccl[2].size.x * ccl[2].size.y);
	int T = 0;
	if(ratio[1] < ratio[T]) T = 1;
	if(ratio[2] < ratio[T]) T = 2;
	int L = (T+1)%3;
	int R = (T+2)%3;
	Vec2d vecR = ccl[R].center - ccl[T].center;
	Vec2d vecL = ccl[L].center - ccl[T].center;
	if(vecR[0]*vecL[1] - vecR[1]*vecL[0] < 0) {
		L = (T+2)%3;
		R = (T+1)%3;
	}

	// calc x,y,yaw
	Vec2d ey(0,-1);
	Vec2d center = Vec2d((ccl[T].center + ccl[L].center + ccl[R].center) / 3);
	Vec2d Xmapped = (ccl[R].center - ccl[L].center); 
	Vec2d Ymapped = (ccl[T].center*2 - ccl[L].center -ccl[R].center) / sqrt(3);
	double yaw = acos(ey.dot(Ymapped)/norm(Ymapped));
	if(0<Ymapped[0]) yaw *= -1;


	// ##### calc X,Y,Z axis #####
	double sizeT = ccl[T].size.x * ccl[T].size.y;
	double sizeL = ccl[L].size.x * ccl[L].size.y;
	double sizeR = ccl[R].size.x * ccl[R].size.y;

	// calc xz
	double equ11 = pow(Xmapped[0], 2) + pow(Xmapped[1], 2) - pow(Ymapped[0], 2) -pow(Ymapped[1], 2);
	double equ12 = Xmapped[0]*Ymapped[0] + Xmapped[1]*Ymapped[1];
	double xz = sqrt(sqrt(equ12*equ12+equ11*equ11/4)-equ11/2);
	if(sizeR < sizeL) xz *= -1;
	//double yzd = - equ12 / xz;

	// calc yz
	double equ21 = pow(Ymapped[0], 2) + pow(Ymapped[1], 2) - pow(Xmapped[0], 2) -pow(Xmapped[1], 2);
	double equ22 = Ymapped[0]*Xmapped[0] + Ymapped[1]*Xmapped[1];
	double yz = sqrt(sqrt(equ22*equ22+equ21*equ21/4)-equ21/2);
	if(sizeT*2 < sizeL+sizeR) yz *= -1;
	//double xzd = - equ22 / yz;


	// ##### calc X,Y,Z axis on image #####
	Vec3d Xaxis(Xmapped[0], Xmapped[1], xz);
	Vec3d Yaxis(Ymapped[0], Ymapped[1], yz);
	Vec3d Zaxis = Yaxis.cross(Xaxis);
	Zaxis = Zaxis / sqrt(norm(Zaxis));
	Vec2d Zmapped(Zaxis[0], Zaxis[1]);

	// normalization
	double axis_norm = (norm(Xaxis) + norm(Yaxis) + norm(Zaxis)) /3;
	Xaxis /= axis_norm;
	Yaxis /= axis_norm;
	Zaxis /= axis_norm;

	// calc roll,pitch,yaw
	double beta = asin(-Xaxis[2]);
	double alpha = atan2(Xaxis[1], Xaxis[0]);
	double gamma = atan2(Yaxis[2], Zaxis[2]);

	// ##### calc object pose #####
	Vec2d cc = tfcv2center(center, Vec2d(image.cols/2, image.rows/2));
	object_pose[0] = cc[0] * sidelen / axis_norm;
	object_pose[1] = cc[1] * sidelen / axis_norm;
	object_pose[2] = yaw;

	//object_pose[0] *= -1;
	//object_pose[2] *= -1;


	// draw axis and circle
	double radius = axis_norm * 0.1;
	if(radius<1) radius = 2;
	circle(image, ccl[T].center, radius, Scalar(255,255,0), 2);
	circle(image, ccl[L].center, radius, Scalar(255,255,100), -1);
	circle(image, ccl[R].center, radius, Scalar(255,255,200), -1);
	arrow(image, Point(center), Point(center+Xmapped), Scalar(100,100,255));
	arrow(image, Point(center), Point(center+Ymapped), Scalar(100,255,100));
	arrow(image, Point(center), Point(center+Zmapped), Scalar(255,100,100));
}

void arrow(Mat& img, Point bottom, Point top, Scalar color, int thickness, int arrowtop_size)
{
	// calc arrow
	Vec2d vec0 = Vec2i(bottom-top);
	Vec2d vec1 = vec_rotation(vec0, M_PI/6) / norm(vec0) * arrowtop_size;
	Vec2d vec2 = vec_rotation(vec0,-M_PI/6) / norm(vec0) * arrowtop_size;
	// draw arrow
	line(img, top, top + Point(vec0), color, thickness);
	line(img, top, top + Point(vec1), color, thickness);
	line(img, top, top + Point(vec2), color, thickness);
}

Vec2d tfcv2center(Vec2d input, Vec2d center, bool inv)
{
	Mat reverse_y = (Mat_<double>(2,2) << 1, 0, 0,-1);
	Vec2d ans;
	if(!inv) ans = (Vec2d)Mat1d(reverse_y * Mat1d(input - center));
	else ans = (Vec2d)Mat1d(reverse_y * Mat1d(input) + Mat1d(center));
	return ans;
}

Vec2d vec_rotation(Vec2d input, double rad)
{
	Mat vec_rotation_mat = (Mat_<double>(2,2) << cos(rad), -sin(rad), sin(rad), cos(rad));
	Vec2d ans = (Vec2d)Mat1d(vec_rotation_mat * Mat1d(input));
	return ans;
}

void draw_text(Vec3d pose, Mat& image)
{
	Scalar color_string = Scalar(120,120,255);
	int font = FONT_HERSHEY_SIMPLEX;
	double fontsize = 0.8;
	double fontweight = 2.0;
	int x0 = 15;
	int x1 = fontsize * 40;
	int x2 = fontsize * 160;
	int x3 = fontsize * 190;
	int y0 = 20;
	int y1 = y0 + fontsize * 40;
	int y2 = y0 + fontsize * 80;

	char s[3][16];
	sprintf(s[0], "%.4f", pose[0]);
	sprintf(s[1], "%.4f", pose[1]);
	sprintf(s[2], "%.4f", pose[2]);
	putText(image, "X",        Point(x0, y0), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, "y",        Point(x0, y1), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, "[m]",      Point(x1, y0), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, "[m]",      Point(x1, y1), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, "Yaw[rad]", Point(x0, y2), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, ":",        Point(x2, y0), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, ":",        Point(x2, y1), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, ":",        Point(x2, y2), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, s[0],       Point(x3, y0), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, s[1],       Point(x3, y1), font, fontsize, color_string, fontweight, CV_AA);
	putText(image, s[2],       Point(x3, y2), font, fontsize, color_string, fontweight, CV_AA);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		cv_bridge::CvImage cv_img=*cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		result_img = cv_img.image;
		ImgSub = true;
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}	
};

void BroadcastTf(string header, string child, Vec3d pose)
{
	static tf::TransformBroadcaster broadcaster;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(-pose[0], pose[1], 0));   // -pose[0] : view change
	tf::Quaternion quaternion;
	quaternion.setRPY(0, 0, -pose[2]);   // -pose[2] : view change
	transform.setRotation(quaternion);
	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), header, child));
}

void PublishMarkerFlag(ros::Publisher& MarkerFlagPub, bool flag)
{
	std_msgs::Bool msg;
	msg.data = flag;
	MarkerFlagPub.publish(msg);
}

void PublishPose(ros::Publisher& MarkerPosePub, Vec3d pose, string frame_name)
{
	std_msgs::Header header;
	geometry_msgs::Point point;
	geometry_msgs::Quaternion quaternion;
	header.stamp = ros::Time::now();
	header.frame_id = frame_name;
	point.x = pose[0];
	point.y = pose[1];
	point.z = 0.0;
	quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, pose[2]);

	geometry_msgs::PoseStamped msg;
	msg.header = header;
	msg.pose.position = point;
	msg.pose.orientation = quaternion;

	MarkerPosePub.publish(msg);
}

void PublishPoseReversed(ros::Publisher& MarkerPoseReversedPub, Vec3d pose, string frame_name)
{
	std_msgs::Header header;
	geometry_msgs::Point point;
	geometry_msgs::Quaternion quaternion;
	header.stamp = ros::Time::now();
	header.frame_id = frame_name;
	point.x = pose[0];
	point.y = -pose[1];
	point.z = 0.0;
	quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, pose[2]);


	geometry_msgs::PoseStamped msg;
	msg.header = header;
	msg.pose.position = point;
	msg.pose.orientation = quaternion;

	MarkerPoseReversedPub.publish(msg);
}

void PublishResultImg(image_transport::Publisher& ImagePub, Mat& image)
{
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	ImagePub.publish(msg);
}

