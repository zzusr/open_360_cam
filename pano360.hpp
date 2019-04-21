#pragma once
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
class FisheyeUnwrapping
{
public:
	void dev_build(Point2f center, float R, Size framesize, float x_fov = 220) {
		map[0] = Mat::zeros(framesize, CV_32FC1);
		map[1] = Mat::zeros(framesize, CV_32FC1);
		float balance = 0.8;
		auto f = R * 220./90.;
		for (size_t i = 0; i < framesize.height; i++) {
			for (size_t j = 0; j < framesize.width; j++) {
				//
				map[0].at<float>(i, j) = round(sqrt(pow(R, 2) - pow((center.x - i), 2))*(j - center.y) / R + center.y);
				map[1].at<float>(i, j) = (float)(i - center.y)*1.3 + center.y;
			}
		}
		for (size_t i = 0; i < 2; i++)
			map[i].copyTo(_map[i]);
	}
	void setCropParameters(Point2f center, float R, Scalar _crop, Rect roi, Size frame_size) {
		circle_center = center;
		circle_R = R;
		crop = _crop;
		_roi = roi;
		input_size = frame_size;
	}
	void saveProfile(string filename) {
		FileStorage fs;
		fs.open(filename, FileStorage::WRITE);
		fs << "CircleCenter" << circle_center;
		fs << "CircleRadius" << circle_R;
		fs << "CropBorderEx" << crop;
		fs << "CropRect_ROI" << _roi;
		fs << "AxisRemap_X" << map[0];
		fs << "AxisRemap_Y" << map[1];
		fs << "InputSize" << input_size;
		fs.release();
	}
	void readProfile(string filename) {
		FileStorage fs;
		fs.open(filename, FileStorage::READ);
		fs["CircleCenter"] >> circle_center;
		fs["CircleRadius"] >> circle_R;
		fs["CropBorderEx"] >> crop;
		fs["CropRect_ROI"] >> _roi;
		fs["AxisRemap_X"] >> map[0];
		fs["AxisRemap_Y"] >> map[1];
		fs["InputSize"] >> input_size;
		fs.release();
		for (size_t i = 0; i < 2; i++)
			map[i].copyTo(_map[i]);
		//dev_build(circle_center, circle_R, input_size);
	}
	UMat Apply(UMat frame, bool need_aglin = true) {
		TickMeter tm;
		tm.reset();
		tm.start();
		UMat _frame;
		if (need_aglin) {
			copyMakeBorder(frame, _frame,
				crop[0]>0 ? crop[0] : 0,
				crop[1]>0 ? crop[1] : 0,
				crop[2]>0 ? crop[2] : 0,
				crop[3]>0 ? crop[3] : 0,
				BORDER_CONSTANT);
			Point ptl(crop[2] < 0 ? -crop[2] : 0, crop[0] < 0 ? -crop[0] : 0),
				pbr(Point(_frame.cols + (crop[3] < 0 ? circle_R + circle_center.x - _frame.cols : ptl.x),
					_frame.rows + (crop[1] < 0 ? circle_R + circle_center.y - _frame.rows : ptl.y)));
			if (pbr.x > _frame.cols)pbr.x = _frame.cols;
			if (pbr.y > _frame.rows)pbr.y = _frame.rows;
			Rect roi(ptl, pbr);
			//rectangle(_frame, roi, Scalar(255, 255, 255));
			_frame(roi).copyTo(_frame);
		}
		else frame.copyTo(_frame);
		remap(_frame.clone(), _frame, _map[0], _map[1], INTER_LINEAR);
		tm.stop();
		//cout << "remap time: " << tm.getTimeMilli() << endl;
		return _frame;
	}
private:
	Mat map[2];
	UMat _map[2];
	Scalar crop;
	Rect _roi;
	float circle_R;
	Point2f circle_center;
	Size input_size;
};


void mouse_callback(int e, int x, int y, int flags, void *param);
class FisheyeUnwrappingWizardApp
{
public:
	void run(string img_filepath, string profile_filename, bool pyrdown = false) {
		for (size_t i = 0; i < 3; i++) {
			circle_pt[i] = Point2d(-1, -1);
		}
		namedWindow("src");
		setMouseCallback("src", mouse_callback, this);
		/*VideoCapture cam(1);
		cam.set(CV_CAP_PROP_FRAME_WIDTH£¬)*/
		Mat src = imread(img_filepath);
		if (pyrdown) pyrDown(src, src);
		Mat canvas;
		for (; status_i < 3; waitKey(1)) {
			canvas = src.clone();
			for (size_t i = 0; i < 3; i++) circle(canvas, circle_pt[i], 5, Scalar(255, 255, 0), -1);
			imshow("src", canvas);
		}
		for (size_t i = 0; i < 3; i++) circle(canvas, circle_pt[i], 5, Scalar(255, 255, 0), -1);
		imshow("src", canvas);
		CircleFrom3pt c(circle_pt[0], circle_pt[1], circle_pt[2]);
		circle(canvas, c.queryCenter(), c.queryR(), Scalar(255, 0, 255), 1);
		Mat mask = Mat::zeros(src.size(), CV_8U);
		circle(mask, c.queryCenter(), c.queryR(), Scalar(255), -1);
		UMat dst = UMat::zeros(Size(c.queryR() * 2 + 1, c.queryR() * 2 + 1), CV_8UC3);
		Scalar crop(
			(c.queryR() - c.queryCenter().y),
			(c.queryR() + c.queryCenter().y - src.rows),
			(c.queryR() - c.queryCenter().x),
			(c.queryR() + c.queryCenter().x - src.cols));
		cout << crop << endl;

		copyMakeBorder(src, dst,
			crop[0]>0 ? crop[0] : 0,
			crop[1]>0 ? crop[1] : 0,
			crop[2]>0 ? crop[2] : 0,
			crop[3]>0 ? crop[3] : 0,
			BORDER_CONSTANT);
		Point ptl(crop[2] < 0 ? -crop[2] : 0, crop[0] < 0 ? -crop[0] : 0),
			pbr(Point(dst.cols + (crop[3] < 0 ? c.queryR() + c.queryCenter().x - dst.cols : ptl.x),
				dst.rows + (crop[1] < 0 ? c.queryR() + c.queryCenter().y - dst.rows : ptl.y)));
		if (pbr.x > dst.cols)pbr.x = dst.cols;
		if (pbr.y > dst.rows)pbr.y = dst.rows;
		Rect roi(ptl, pbr);
		//rectangle(dst, roi, Scalar(255, 255, 255));
		dst(roi).clone().copyTo(dst);
		cout << dst.size() << endl << roi << endl;
		imshow("cropped", dst);
		waitKey(1);
		/*

		Point p1(dst.cols / 2, dst.rows / 2), p2(c.queryCenter()),p((p1 + p2) / 2);
		src(roi).copyTo(dst(Rect(p.x - roi.width / 2, p.y - roi.height / 2, roi.width, roi.height)));*/
		imshow("src", canvas);
		circle(dst, dst.size() / 2, c.queryR(), Scalar(255), 1);
		imshow("cropped", dst);
		waitKey(1);
		FisheyeUnwrapping fu;
		fu.setCropParameters(c.queryCenter(), c.queryR(), crop, roi, dst.size());
		fu.dev_build(Point2f(dst.cols / 2, dst.rows / 2), c.queryR(), dst.size());
		dst = fu.Apply(src.getUMat(ACCESS_RW));
		fu.saveProfile(profile_filename);
		imshow("dst", dst);
		waitKey();
		destroyAllWindows();
		status_i = 0;
	}
	Point mouse_pt;
	Point2d circle_pt[3];
	int status_i = 0;
private:
	class CircleFrom3pt
	{
	public:
		CircleFrom3pt(Point2d pt1, Point2d pt2, Point2d pt3) {
			p1 = pt1;
			p2 = pt2;
			p3 = pt3;
			m1 = midpoint(p1, p2);
			m2 = midpoint(p2, p3);
			m3 = midpoint(p3, p1);
			l1 = line2p(p1, p2);
			l2 = line2p(p2, p3);
			l3 = line2p(p3, p1);
			lm1 = linepoint(m1, l1.k, pai / 2);
			lm2 = linepoint(m2, l2.k, pai / 2);
			lm3 = linepoint(m3, l3.k, pai / 2);
			circlep = crosspoint(lm1, lm2);
			circler = dist(circlep, p1);
			dis2 = dist(circlep, p2);
			dis3 = dist(circlep, p3);
		}
		Point2f queryCenter() {
			return circlep;
		}
		double queryR() {
			return circler;
		}
	private:
		double const pai = 3.1415926535897932384626;

		struct line {
			double k = 0;    //Ð±ÂÊ
			double b = 0;    //½Ø¾à
		};

		double dist(cv::Point2d p1, cv::Point2d p2) {
			double ret = 0;
			ret = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
			return ret;
		}

		cv::Point2d midpoint(cv::Point2d p1, cv::Point2d p2) {
			return Point2d((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
		}

		line line2p(cv::Point2d p1, cv::Point2d p2) {
			line ret;
			if ((p1.x == p2.x) && (p1.y = p2.y)) return ret;
			ret.k = (p1.y - p2.y) / (p1.x - p2.x);
			ret.b = p1.y - p1.x * ret.k;
			return ret;
		}

		bool iscross(line l1, line l2) {
			return !(l1.k == l2.k);
		}

		cv::Point2d crosspoint(line l1, line l2)
		{
			cv::Point2d ret;
			if (!iscross(l1, l2)) return ret;
			ret.x = (l2.b - l1.b) / (l1.k - l2.k);
			ret.y = (l1.k*l2.b - l2.k*l1.b) / (l1.k - l2.k);
			return ret;
		}

		line linepoint(cv::Point2d p, double k, double n) {
			line ret;
			ret.k = tan(atan(k) + n);
			ret.b = p.y - ret.k * p.x;
			return ret;
		}

		cv::Point2d p1, p2, p3;
		cv::Point2d m1, m2, m3;
		line l1, l2, l3;
		line lm1, lm2, lm3;
		cv::Point2d circlep;
		double circler, dis2, dis3;
	};
};
void mouse_callback(int e, int x, int y, int flags, void *param)
{
	auto instance = (FisheyeUnwrappingWizardApp*)param;
	if (instance->status_i >= 3) return;
	switch (e)
	{
	case EVENT_LBUTTONUP:
		instance->circle_pt[instance->status_i] = Point2d(x, y);
		instance->status_i++;
		break;
	default:
		break;
	}
}
class stitchingBlender
{
public:
	void set_params(UMat &left, UMat &right, int width) {
		sz_l = left.size();
		sz_r = right.size();
		band_width = width;
	}
	UMat blend_2(UMat left, UMat right, int width = 20) {
		UMat blend = UMat::zeros(Size(left.cols + right.cols - width, max(left.rows, right.rows)), CV_8UC3);
		MakeTransition(left, 0, width).copyTo(blend(Rect(0, 0, left.cols, left.rows)));
		addWeighted(MakeTransition(right, width, 0), 1, blend(Rect(left.cols - width, 0, right.cols, right.rows)), 1, 0, blend(Rect(left.cols - width, 0, right.cols, right.rows)));
		return blend;
	}

	UMat MakeTransition(UMat src, int left, int right) {
		Mat swap = src.getMat(ACCESS_READ);
		for (size_t i = 0; i < left; i++) {
			float balance = i / (float)left;
			swap.col(i) *= balance;
		}
		for (size_t i = src.cols - right; i < src.cols; i++) {
			float balance = float(src.cols - i) / right;
			swap.col(i) *= balance;
		}
		return swap.getUMat(ACCESS_RW);
	}
	void build_trans() {
		trans_l = UMat(sz_l.height, band_width, CV_8UC3);
		trans_r = UMat(sz_l.height, band_width, CV_8UC3);
		for (size_t i = 0; i < band_width; i++)
		{

		}
	}
private:
	UMat trans_l, trans_r;
	Size sz_l, sz_r;
	int band_width;
};