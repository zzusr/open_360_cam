#include<opencv2\opencv.hpp>
#include<Windows.h>
#include<opencv2\dnn.hpp>
#include"pano360_camera.hpp"
#include "USART.h"
#include "BaseMoveComm.h"
using namespace std;
using namespace cv;
using namespace dnn;
#define devView(i) imshow(#i,i)
#define _exposure -8
VideoCapture create_cam(int idx) {
	VideoCapture cam(idx);
	cam.set(CAP_PROP_FOURCC, 'GPJM');
	cam.set(CAP_PROP_FRAME_WIDTH, 1600);
	cam.set(CAP_PROP_FRAME_HEIGHT, 1200);
	Sleep(1000);
	return cam;
}
Comm com1;
unsigned char Send_Buff[160];
string str;
int m_angle_real = 90;
int m_angle_temp = 0;

const size_t inWidth = 300;
const size_t inHeight = 300;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;

DWORD WINAPI RevProcAngle(LPVOID pParam)
{
	while (1)
	{
		if (abs(m_angle_real - 90) >20)
		{
			int m_rotateangle = 0;
			int m_delayms = 0;
			unsigned char Send_Buff[200];
			m_angle_temp = m_angle_real;
			std::cout << "角度:" << m_angle_temp << std::endl;
			if ((m_angle_temp < 90) && (m_angle_temp >= 0))
			{
				m_angle_temp = 90 - m_angle_temp;
				BaseMoveControl(Send_Buff, 0, 520, 0);//30度/s
				com1.WriteChar(Send_Buff, 16);
				m_rotateangle = abs(m_angle_temp);  //逆时针旋转m_rotateangle
				m_delayms = m_rotateangle * 1000 / 30;

				std::cout << "逆时针旋转：" << m_rotateangle << std::endl;
				std::cout << m_delayms << std::endl;
				Sleep(m_delayms);
				BaseMoveControl(Send_Buff, 0, 0, 0);
				com1.WriteChar(Send_Buff, 16);
			}
			if ((m_angle_temp < 0) && (m_angle_temp >= -90))
			{
				m_angle_temp = 90 - m_angle_temp;
				BaseMoveControl(Send_Buff, 0, 520, 0);//30度/s
				com1.WriteChar(Send_Buff, 16);
				m_rotateangle = abs(m_angle_temp);  //逆时针旋转m_rotateangle
				m_delayms = m_rotateangle * 1000 / 30;

				std::cout << "逆时针旋转：" << m_rotateangle << std::endl;
				std::cout << m_delayms << std::endl;
				Sleep(m_delayms);
				BaseMoveControl(Send_Buff, 0, 0, 0);
				com1.WriteChar(Send_Buff, 16);
			}
			if ((m_angle_temp >= 90) && (m_angle_temp < 180))
			{
				m_angle_temp = 90 - m_angle_temp;
				BaseMoveControl(Send_Buff, 0, -520, 0);//30度/s
				com1.WriteChar(Send_Buff, 16);
				m_rotateangle = abs(m_angle_temp);  //顺时针旋转m_rotateangle
				m_delayms = m_rotateangle * 1000 / 30;

				std::cout << "顺时针旋转：" << m_rotateangle << std::endl;
				std::cout << m_delayms << std::endl;
				Sleep(m_delayms);
				BaseMoveControl(Send_Buff, 0, 0, 0);
				com1.WriteChar(Send_Buff, 16);
			}
			if ((m_angle_temp >= -180) && (m_angle_temp < -90))
			{
				m_angle_temp = -270 - m_angle_temp;
				BaseMoveControl(Send_Buff, 0, -520, 0);//30度/s
				com1.WriteChar(Send_Buff, 16);
				m_rotateangle = abs(m_angle_temp);  //顺时针旋转m_rotateangle
				m_delayms = m_rotateangle * 1000 / 30;

				std::cout << "顺时针旋转：" << m_rotateangle << std::endl;
				std::cout << m_delayms << std::endl;
				Sleep(m_delayms);
				BaseMoveControl(Send_Buff, 0, 0, 0);
				com1.WriteChar(Send_Buff, 16);
			}
			m_angle_temp = 0;
			Sleep(20);
		}
		else
		{
			Sleep(10);
		}
	}
	//get_angle_flag = true;
	//return get_angle_flag;
}

int main()
{	
	com1.openport(L"COM3", 115200);
	PurgeComm(com1.hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	BaseMoveControl(Send_Buff, 0, 0, 0);
	com1.WriteChar(Send_Buff, 20);
	HANDLE RevThread = CreateThread(NULL, 0, RevProcAngle, NULL, 0, NULL);
	CloseHandle(RevThread);
	// capture init
	VideoCapture cam_fisheye[2] = { create_cam(1),create_cam(0) };
	UMat frame_fisheye[2];
	for (size_t i = 0; i < 2; i++)	cam_fisheye[i] >> frame_fisheye[i];
	// fisheye init
	FisheyeUnwrapping un[2];
	un[0].readProfile("b.xml");
	un[1].readProfile("a.xml");
	stitchingBlender blender;
	// loop
	for (int key = 0; (key = waitKey(1)) != 27;)
	{
		// panoimg proc
		for (size_t i = 0; i < 2; i++) 
		{ 
			cam_fisheye[i].set(CAP_PROP_EXPOSURE, _exposure);
			cam_fisheye[i] >> frame_fisheye[i];
		}

		UMat fu_frame = blender.blend_2(un[0].Apply(frame_fisheye[0]), un[1].Apply(frame_fisheye[1]), 120);
		int cut = 0.3*fu_frame.rows;
		Mat frame = fu_frame(Rect(0, cut, fu_frame.cols, fu_frame.rows - 2 * cut)).clone().getMat(ACCESS_RW).clone();
		Mat frame_lable = Mat::zeros(frame.size(), CV_8UC3);
		// inference & draw bbox
		String modelConfiguration = "MobileNetSSD_deploy.prototxt";
		String modelBinary = "MobileNetSSD_deploy.caffemodel";
		Net net = readNetFromCaffe(modelConfiguration, modelBinary);

		Mat inputBlob = blobFromImage(frame, inScaleFactor, Size(inWidth, inHeight), Scalar(meanVal, meanVal, meanVal), false, false);
		net.setInput(inputBlob);
		Mat detection = net.forward("detection_out");
		Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

		vector<int> person_lidar_in_degree;
		vector<int> person_width;

		for (int i = 0; i < detectionMat.rows; i++)
		{
			float confidence = detectionMat.at<float>(i, 2);
			size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));
			if ((confidence > 0.4)&&(objectClass == 15))
			{
				int tl_x = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
				int tl_y = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
				int br_x = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
				int br_y = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);
				int width = br_x - tl_x;
				int height = br_y - tl_y;
				if (width > 350) continue;
				if (width < 100) continue;
				int head_margin = 20;
				if (tl_y > head_margin)tl_y -= head_margin;
				else tl_y = 0;
				if (tl_x + width > frame.cols)width = frame.cols - tl_x - 1;
				if (tl_y + height > frame.rows)height = frame.rows - tl_y - 1;
				if (tl_x < 0)tl_x = 0;
				if (tl_y < 0)tl_y = 0;
				person_width.push_back(width);
				int angle_body = (tl_x + width / 2.) / (float)frame.cols * 360 - 180;
				person_lidar_in_degree.push_back(angle_body);
				rectangle(frame, Point(tl_x, tl_y), Point(br_x, br_y), Scalar(255, 0, 0), 3);
				putText(frame, "Angle: " + to_string(angle_body), Point(tl_x + 10, tl_y + 20), 0, 0.8, Scalar(0, 200, 0), 2);
				putText(frame, "Conf: " + to_string(confidence), Point(tl_x + 10, tl_y + 50), 0, 0.8, Scalar(0, 200, 0), 2);
				
			}
		}
		if (person_width.size() > 1)
		{
			std::vector<int>::iterator biggest = std::max_element(std::begin(person_width), std::end(person_width));
			//std::cout << "Max element is " << *biggest << " at position " << std::distance(std::begin(person_width), biggest) << std::endl;
			m_angle_real = person_lidar_in_degree[std::distance(std::begin(person_width), biggest)];
		}
		else
		{
			if (person_width.size() == 1)
				m_angle_real = person_lidar_in_degree[0];
		}
		putText(frame, "Front ", Point(0.75 * frame.cols - 20, 20), 0, 0.8, Scalar(0, 200, 0), 2);
		putText(frame, "back ", Point(0.25 * frame.cols - 20, 20), 0, 0.8, Scalar(0, 200, 0), 2);
		addWeighted(frame, 1, frame_lable, 0.5, 0, frame);
		devView(frame);
	}
}