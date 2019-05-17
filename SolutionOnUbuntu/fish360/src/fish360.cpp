#include "ros/ros.h"
#include <stdlib.h>
#include <string.h>
#include <c++/5.4.0/iostream>
#include <c++/5.4.0/sstream>
#include <c++/5.4.0/string>
#include <c++/5.4.0/stdexcept>
#include <c++/5.4.0/vector>

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <opencv2/dnn.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#define camID 0
#include"pano360_camera.hpp"
#define devView(i) imshow(#i,i)
	
using namespace cv;
using namespace std;
using namespace dnn;

VideoCapture create_cam(int idx) {
	VideoCapture cam(idx);
	cam.set(CAP_PROP_FRAME_WIDTH, 1600);
	cam.set(CAP_PROP_FRAME_HEIGHT, 1200);
	cam.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
	cam.set(CAP_PROP_EXPOSURE, -5);
	return cam;
}

//===============================================================================================================================
const size_t inWidth = 300;
const size_t inHeight = 300;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;

//===============================================================================================================================
//===============================================================================================================================
//===============================================================================================================================

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fish360");
	ros::NodeHandle fish;
	ros::Publisher angle_pub = fish.advertise<std_msgs::Int32>("chatter", 1000);
	std_msgs::Int32 angle_result;
	int angle_now = 0;
	int left = 0;
	int right = 1;
	int exposure = -10;
	
	String modelConfiguration = "/home/robot/catkin_ws/src/fish360/src/MobileNetSSD_deploy.prototxt";
	String modelBinary = "/home/robot/catkin_ws/src/fish360/src/MobileNetSSD_deploy.caffemodel";
	Net net = readNetFromCaffe(modelConfiguration, modelBinary);
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
	VideoCapture cam_fisheye[2] = { create_cam(left),create_cam(right) };
	UMat frame_fisheye[2];
	for (size_t i = 0; i < 2; i++){
		cam_fisheye[i].set(CAP_PROP_EXPOSURE, exposure);
		cam_fisheye[i] >> frame_fisheye[i];
	}
	for (size_t i = 0; i < 2; i++) {
		if(frame_fisheye[i].empty()) cout<<"!!! error cam:"<<i<<endl;
	}
	FisheyeUnwrapping un[2];
	un[0].readProfile("/home/robot/catkin_ws/src/fish360/src/_a.xml");
	un[1].readProfile("/home/robot/catkin_ws/src/fish360/src/_b.xml");
	stitchingBlender blender;
	int count_pub = 0;
	for ( ; 1; count_pub++ ) 
	{
		if(count_pub > 100){ count_pub = 50; }
		
		for (size_t i = 0; i < 2; i++) { 
			cam_fisheye[i].set(CAP_PROP_EXPOSURE, exposure);
			cam_fisheye[i] >> frame_fisheye[i];					
		}
		Mat leftframe, rightframe;
		resize(frame_fisheye[0],leftframe,Size(600,450));
		resize(frame_fisheye[1],rightframe,Size(600,450));
		UMat fu_frame = blender.blend_2(un[0].Apply(frame_fisheye[0]), un[1].Apply(frame_fisheye[1]), 120);
		int cut = 0.3 * fu_frame.rows;
		Mat frame = fu_frame(Rect(0, cut, fu_frame.cols, fu_frame.rows - 2 * cut)).clone().getMat(ACCESS_RW).clone();
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
		Mat inputBlob = blobFromImage(frame, inScaleFactor, Size(inWidth, inHeight), Scalar(meanVal, meanVal, meanVal), false, false);
		net.setInput(inputBlob); 		
		Mat detection = net.forward("detection_out"); 								 
		Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
		float confidenceThreshold = 0.20;

		vector<int> angle_closer;
		vector<int> person_width;

		for (int i = 0; i < detectionMat.rows; i++){
			float confidence = detectionMat.at<float>(i, 2);
			if (confidence > confidenceThreshold){
				size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));
				int tl_x = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
				int tl_y = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
				int br_x = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
				int br_y = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);
				if(( objectClass==15 )&&( confidence > 0.20 )){
					int angle_p = 360 * 0.5 * (br_x + tl_x) / frame.cols;
					rectangle(frame, Point(tl_x, tl_y), Point(br_x, br_y), Scalar(255, 0, 0), 3);
					putText(frame, to_string(confidence), Point(tl_x, tl_y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 20, 255), 3);
					putText(frame, "Angle:  " + to_string(angle_p), Point(tl_x, tl_y + 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 20, 255), 3);
					if(br_x - tl_x > 90){
						angle_closer.push_back(angle_p);
						person_width.push_back(br_x - tl_x);
					}
				}	
			}
		}

		if (person_width.size() >= 1)
		{
			std::vector<int>::iterator biggest = std::max_element(std::begin(person_width), std::end(person_width));
			angle_now = angle_closer[std::distance(std::begin(person_width), biggest)];
			
			if((80 < angle_now)&&(angle_now < 100))
			{
				angle_result.data = 0;
			}
			else
			{
				if((0 <= angle_now)&&(angle_now < 90))
				{
					angle_now = 90 - angle_now;
				}
				if((90 <= angle_now)&&(angle_now < 180))
				{
					angle_now = 90 - angle_now;
				}
				if((180 <= angle_now)&&(angle_now < 270))
				{
					angle_now = 90 - angle_now;
				}
				if((270 <= angle_now)&&(angle_now <= 360))
				{
					angle_now = 90 + 360 - angle_now;
				}
				angle_result.data = angle_now;
			}
			
			if(count_pub > 40)
			{
				angle_pub.publish(angle_result);
				count_pub = 0;
			}
		}
		
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

		
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
		Mat frame_lable = Mat::zeros(frame.size(), CV_8UC3);
		addWeighted(frame, 1, frame_lable, 0.5, 0, frame);
		resize(frame, frame, Size(1600, 400));
		
		Mat canvas = Mat::zeros(Size(frame.cols, frame.rows+leftframe.rows), CV_8UC3);
		Rect roi(0, 0, frame.cols, frame.rows);
		frame.copyTo(canvas(roi));
		leftframe.copyTo(canvas(Rect(0,frame.rows,leftframe.cols,leftframe.rows)));
		rightframe.copyTo(canvas(Rect(frame.cols - rightframe.cols,frame.rows,rightframe.cols,rightframe.rows)));
		putText(canvas, "Closest human", Point(0.5*canvas.cols-200, 0.5*canvas.rows+100), FONT_HERSHEY_SIMPLEX, 0.9, Scalar(0, 0, 255), 2);
		putText(canvas, "angle  :   " + to_string(angle_result.data), Point(0.5*canvas.cols-200, 0.5*canvas.rows+200), FONT_HERSHEY_SIMPLEX, 0.9, Scalar(0, 0, 255), 2);
		imshow("", canvas);
		if(waitKey(30)==27)break;
	}

	return 0;
}

