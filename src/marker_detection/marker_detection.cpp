#include <marker_detection/marker_detection.h>
using namespace std;

void keychangeHSV(vector<HSV>& list)
{
	static int n = 0;
	enum {H_UP='q', H_DN='a', R_UP='w', R_DN='s', S_UP='e', S_DN='d', V_UP='r', V_DN='f', N_UP='t', N_DN='g'};
	if (kbhit()) {
		short int key;
		key = getchar();
		switch(key){
			case H_UP: list[n].hue += 2; if (list[n].hue > 179) list[n].hue -= 180; break;
			case H_DN: list[n].hue -= 2; if (list[n].hue <   0) list[n].hue += 180; break;
			case R_UP: list[n].range += 2; break;
			case R_DN: list[n].range -= 2; break;
			case S_UP: list[n].sat += 2; break;
			case S_DN: list[n].sat -= 2; break;
			case V_UP: list[n].val += 2; break;
			case V_DN: list[n].val -= 2; break;
			case N_UP: n++; if (list.size()<=n) n = list.size()-1; break;
			case N_DN: n--; if (n<0) n = 0; break;
			default:;
		}
		cout<<"list["<<n<<"] : "<<" hue:"<<list[n].hue<<" range:"<<list[n].range<<" sat:"<<list[n].sat<<" val:"<<list[n].val<<endl;
	}
}

void color_extract(cv::Mat input_img, cv::Mat& output_img, vector<HSV> list)
{	
	output_img = cv::Mat::zeros(cv::Size(input_img.cols, input_img.rows), CV_8UC1);
	cv::Mat blur_img;
	cv::GaussianBlur(input_img, blur_img, cv::Size(9,9), 0, 0);
	cv::cvtColor(blur_img, blur_img, CV_BGR2HSV);
	for(int y=0; y<blur_img.rows; y++){
		cv::Vec3b *src_i = blur_img.ptr<cv::Vec3b>(y);
		for(int x=0; x<blur_img.cols; x++){
			for(int i=0; i<list.size(); i++) {
				int huemin = (list[i].hue - list[i].range/2)%180;
				int huemax = (list[i].hue + list[i].range/2)%180;
				if(list[i].sat < src_i[x][1] && list[i].val < src_i[x][2] &&  huemin< src_i[x][0] && src_i[x][0] < huemax) {
					output_img.at<unsigned char>(y,x) = list[i].set;
				}
			}
		}
	}
}

void label(cv::Mat binary_img, vector<Regiondata>& regiondata, unsigned int remove_pix)
{
	regiondata.clear();
	cv::Mat label(binary_img.size(), CV_16SC1);
	LabelingBS labeling;
	labeling.Exec(binary_img.data, (short *)label.data, binary_img.cols, binary_img.rows, true, remove_pix);	
	RegionInfoBS *ri;
	for( int i = 0; i < labeling.GetNumOfResultRegions(); i++)
	{
		Regiondata now;
		ri = labeling.GetResultRegionInfo(i);
		//cv::Mat labelarea;
		//cv::compare(label, i+1, labelarea, CV_CMP_EQ);
		// fighting!!! cv::minEnclosingCircle(labelarea, data[i].c_center, data[i].radius);
		//cv::Mat color(binary_img.size(), CV_8UC3, cv::Scalar(255,255,255));
		//color.copyTo(label_img, labelarea);
		now.pixels = ri->GetNumOfPixels(); 
		cv::Point2f center;
		ri->GetCenter(center.x, center.y); 
		now.center.x = center.x;
		now.center.y = center.y;
		ri->GetSize(now.size.x, now.size.y);
		ri->GetMin(now.min.x, now.min.y);
		ri->GetMax(now.max.x, now.max.y);
		now.color = ri->GetSourceValue();
		regiondata.push_back(now);
	}
	sort(regiondata.begin(), regiondata.end());
}

bool markerpose_detection(cv::Vec3d& markerpose, vector<Regiondata> regiondata, cv::Mat& image, double sidelen)
{
	bool marker_exist = false;
	vector<cv::Vec2d> marker_circle;
	int num1=-1, num2=-1, num3=-1;
	double normmin = -1.0;
	Regiondata comp[3];
	for(int i=0; i<regiondata.size(); i++) {
		for(int j=i+1; j<regiondata.size(); j++) {
			for(int k=j+1; k<regiondata.size(); k++) {
				comp[0] = regiondata[i];
				comp[1] = regiondata[j];
				comp[2] = regiondata[k];
				if(samesize(comp) && triangle(comp)) {
					double normsum = 0.0;
					for(int l=0; l<3; l++) {
						cv::Vec2d vec = cv::Vec2d(comp[l].center - comp[(l+1)%3].center);
						normsum += cv::norm(vec);
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
	}
	//Regiondata comp[3] = {regiondata[i], regiondata[i+1], regiondata[i+2]};
	if(num1 != -1 && num2 != -1 && num3 != -1) {
		comp[0] = regiondata[num1];
		comp[1] = regiondata[num2];
		comp[2] = regiondata[num3];
		calc_markerpose(markerpose, comp, image, sidelen);
		marker_exist = true;
		cout<<"marker exist."<<endl<<endl;
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
	cout<<"size : "<<ans<<endl;
	return ans;
}

bool triangle(Regiondata* region, double threshold)
{
	bool ans = true;
	for(int i=0; i<3; i++) {
		double norm_v1 = cv::norm(cv::Vec2d(
					region[i].center.x-region[(i+1)%3].center.x,
					region[i].center.y-region[(i+1)%3].center.y));
		double norm_v2 = cv::norm(cv::Vec2d(
					region[(i+1)%3].center.x-region[(i+2)%3].center.x,
					region[(i+1)%3].center.y-region[(i+2)%3].center.y));
		if ( fabs(1-atan2(norm_v1, norm_v2)) > threshold ) {
			ans = false;
			break;
		}
	}
	cout<<"triangle : "<<ans<<endl;
	return ans;
}

bool colorcheck(Regiondata* region, cv::Mat& image)
{
	bool ans = false;
	int top = -1;
	int bottom = 0;
	for(int i=0; i<3; i++) {
		cout<<"region(y,x) = "<<region[i].center.y<<","<<region[i].center.x<<endl;
		unsigned char centercolor = image.at<unsigned char>(region[i].center.y, region[i].center.x);
		//if(centercolor == 0) top = i;
		//if(centercolor == BLUE) bottom++;
	}
	cout<<"top : bottom = "<<top<<" : "<<bottom<<endl;
	if(top!=-1 && bottom==2) {
		ans = true;
		//for(int i=0; i<3; i++) {
		//	cv::Vec2d circle(region[(i+green)%3].center.x, region[(i+green)%3].center.y);
		//	circles.push_back(circle);
		//}
		//if(circles[1][0]*circles[2][1]-circles[1][1]*circles[2][0] > 0) {
		//	swap(circles[1], circles[2]);
		//}
	}
	return ans;
}

void calc_markerpose(cv::Vec3d& object_pose, Regiondata* ccl, cv::Mat& image, double sidelen)
{
	double ratio[3];
	ratio[0] = (double)ccl[0].pixels / (ccl[0].size.x * ccl[0].size.y);
	ratio[1] = (double)ccl[1].pixels / (ccl[1].size.x * ccl[1].size.y);
	ratio[2] = (double)ccl[2].pixels / (ccl[2].size.x * ccl[2].size.y);
	int T = 0;
	if(ratio[1] < ratio[T]) T = 1;
	if(ratio[2] < ratio[T]) T = 2;
	int L = (T+1)%3;
	int R = (T+2)%3;
	cv::Vec2d vecR = ccl[R].center - ccl[T].center;
	cv::Vec2d vecL = ccl[L].center - ccl[T].center;
	if(vecR[0]*vecL[1] - vecR[1]*vecL[0] < 0) {
		L = (T+2)%3;
		R = (T+1)%3;
	}

	cv::Vec2d ex(0,-1);
	cv::Vec2d center = cv::Vec2d((ccl[T].center + ccl[L].center + ccl[R].center) / 3);
	cv::Vec2d Xmapped = (ccl[R].center - ccl[L].center); 
	cv::Vec2d Ymapped = (ccl[T].center*2 - ccl[L].center -ccl[R].center) / sqrt(3);
	double yaw = acos(ex.dot(Ymapped)/cv::norm(Ymapped));
	if(0<Ymapped[0]) yaw *= -1;

	double sizeT = ccl[T].size.x * ccl[T].size.y;
	double sizeL = ccl[L].size.x * ccl[L].size.y;
	double sizeR = ccl[R].size.x * ccl[R].size.y;
	
	// calc xz
	double equ11 = pow(Xmapped[0], 2) + pow(Xmapped[1], 2) - pow(Ymapped[0], 2) -pow(Ymapped[1], 2);
	double equ12 = Xmapped[0]*Ymapped[0] + Xmapped[1]*Ymapped[1];
	double xz = sqrt(sqrt(equ12*equ12+equ11*equ11/4)-equ11/2);
	if(sizeR < sizeL) xz *= -1;
	//double yzd = - equ12 / xz;
	
	//calc yz
	double equ21 = pow(Ymapped[0], 2) + pow(Ymapped[1], 2) - pow(Xmapped[0], 2) -pow(Xmapped[1], 2);
	double equ22 = Ymapped[0]*Xmapped[0] + Ymapped[1]*Xmapped[1];
	double yz = sqrt(sqrt(equ22*equ22+equ21*equ21/4)-equ21/2);
	if(sizeT*2 < sizeL+sizeR) yz *= -1;
	//double xzd = - equ22 / yz;

	//xz = (xz*3+xzd)/4;
	//yz = (yz*3+yzd)/4;
	
	cv::Vec3d Xaxis(Xmapped[0], Xmapped[1], xz);
	cv::Vec3d Yaxis(Ymapped[0], Ymapped[1], yz);
	cv::Vec3d Zaxis = Yaxis.cross(Xaxis);
	Zaxis = Zaxis / sqrt(cv::norm(Zaxis));
	cv::Vec2d Zmapped(Zaxis[0], Zaxis[1]);
	double axis_norm = (cv::norm(Xaxis) + cv::norm(Yaxis) + cv::norm(Zaxis)) /3;

	cout<<"equ11 : "<<equ11<<endl;
	cout<<"equ12 : "<<equ12<<endl;
	cout<<"Xaxis : "<<Xaxis<<endl;
	cout<<"Yaxis : "<<Yaxis<<endl;
	cout<<"Zaxis : "<<Zaxis<<endl;

	// normalization
	Xaxis /= axis_norm;
	Yaxis /= axis_norm;
	Zaxis /= axis_norm;

	double beta = asin(-Xaxis[2]);
	double alpha = atan2(Xaxis[1], Xaxis[0]);
	double gamma = atan2(Yaxis[2], Zaxis[2]);

	cv::Vec2d cc = tfcv2center(center, cv::Vec2d(image.cols/2, image.rows/2));
	object_pose[0] = cc[0] * sidelen / axis_norm;
	object_pose[1] = cc[1] * sidelen / axis_norm;
	object_pose[2] = yaw;

	object_pose[0] *= -1;
	object_pose[2] *= -1;

	cout<<"calc finish"<<endl;

	double radius = axis_norm * 0.1;
	if(radius<1) radius = 2;
	cv::circle(image, ccl[T].center, radius, cv::Scalar(255,255,0), 2);
	cv::circle(image, ccl[L].center, radius, cv::Scalar(255,255,100), -1);
	cv::circle(image, ccl[R].center, radius, cv::Scalar(255,255,200), -1);
	arrow(image, cv::Point(center), cv::Point(center+Xmapped), cv::Scalar(100,100,255));
	arrow(image, cv::Point(center), cv::Point(center+Ymapped), cv::Scalar(100,255,100));
	arrow(image, cv::Point(center), cv::Point(center+Zmapped), cv::Scalar(255,100,100));
}

void arrow(cv::Mat img, cv::Point bottom, cv::Point top, CvScalar color, int thickness, int arrowtop_size)
{
	// calc arrow
	cv::Vec2d vec0 = cv::Vec2i(bottom-top);
	cv::Vec2d vec1 = vec_rotation(vec0, M_PI/6) / cv::norm(vec0) * arrowtop_size;
	cv::Vec2d vec2 = vec_rotation(vec0,-M_PI/6) / cv::norm(vec0) * arrowtop_size;
	// draw arrow
	cv::line(img, top, top + cv::Point(vec0), color, thickness);
	cv::line(img, top, top + cv::Point(vec1), color, thickness);
	cv::line(img, top, top + cv::Point(vec2), color, thickness);
}

cv::Vec2d tfcv2center(cv::Vec2d input, cv::Vec2d center, bool inv)
{
	cv::Mat reverse_y = (cv::Mat_<double>(2,2) << 1, 0, 0,-1);
	cv::Vec2d ans;
	if(!inv) ans = (cv::Vec2d)cv::Mat1d(reverse_y * cv::Mat1d(input - center));
	else ans = (cv::Vec2d)cv::Mat1d(reverse_y * cv::Mat1d(input) + cv::Mat1d(center));
	return ans;
}

cv::Vec2d vec_rotation(cv::Vec2d input, double rad)
{
	cv::Mat vec_rotation_mat = (cv::Mat_<double>(2,2) << cos(rad), -sin(rad), sin(rad), cos(rad));
	cv::Vec2d ans = (cv::Vec2d)cv::Mat1d(vec_rotation_mat * cv::Mat1d(input));
	return ans;
}

void draw_text(cv::Vec3d pose, cv::Mat& image)
{
	cv::Scalar color_string = cv::Scalar(120,120,255);
	int font = cv::FONT_HERSHEY_SIMPLEX;
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
	sprintf(s[0], "%.7f", pose[0]);
	sprintf(s[1], "%.7f", pose[1]);
	sprintf(s[2], "%.7f", pose[2]);
	cv::putText(image, "X",        cv::Point(x0, y0), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, "y",        cv::Point(x0, y1), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, "[m]",      cv::Point(x1, y0), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, "[m]",      cv::Point(x1, y1), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, "Yaw[rad]", cv::Point(x0, y2), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, ":",        cv::Point(x2, y0), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, ":",        cv::Point(x2, y1), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, ":",        cv::Point(x2, y2), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, s[0],       cv::Point(x3, y0), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, s[1],       cv::Point(x3, y1), font, fontsize, color_string, fontweight, CV_AA);
	cv::putText(image, s[2],       cv::Point(x3, y2), font, fontsize, color_string, fontweight, CV_AA);
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

void BroadcastTf(string header, string child, cv::Vec3d pose)
{
	static tf::TransformBroadcaster broadcaster;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(pose[0], pose[1], 0));
	tf::Quaternion quaternion;
	quaternion.setRPY(0, 0, pose[2]);
	transform.setRotation(quaternion);
	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), header, child));
}

void PublishMarkerFlag(ros::Publisher& MarkerFlagPub, bool flag)
{
	std_msgs::Bool msg;
	msg.data = flag;
	MarkerFlagPub.publish(msg);
}

void PublishPose(ros::Publisher& MarkerPosePub, cv::Vec3d pose, string frame_name)
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

void PublishResultImg(image_transport::Publisher& ImagePub, cv::Mat& image)
{
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	ImagePub.publish(msg);
}


