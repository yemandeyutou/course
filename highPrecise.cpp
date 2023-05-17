#include"highPrecise.h"
#include<math.h>
#include<iostream>
#include<string>
#include<unordered_map>
#include<algorithm>

using namespace std;
using namespace cv;
//高精度检测器类的实现

const double pi = 3.141592654;//定义π


//标定用 用于计算指针刻度 计算公式为：刻度=值/角度
const double scale1 = 0.3999 / (-16.8256 -(-51.3977));//  value/angle  
const double scale2 = 0.3599 / (-30.4572 - (-53.5387));
//定义线检测时 使用的核大小
const int LINE_KERNAL = 9;

//定义圆检测时 使用的核大小
const int CIRCLE_KERNALVALUE = 11;


//得到存储有仪表盘圆的容器
std::vector<cv::Vec3f> highPreciseDetector::getCircles() {
	
	//将图像转换为灰度单通道图像 
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	//双边滤波 用于去除噪音
	Mat bf;
	bilateralFilter(gray, bf, CIRCLE_KERNALVALUE, CIRCLE_KERNALVALUE * 2, CIRCLE_KERNALVALUE / 2);
	//获取图像边缘
	Mat canny;
	Canny(bf, canny, 10, 250, 5);
	//双边滤波
	Mat hough;
	bilateralFilter(canny, hough, CIRCLE_KERNALVALUE, CIRCLE_KERNALVALUE * 2, CIRCLE_KERNALVALUE / 2);
	//将HoughCircle()检测到的圆存储到circles容器中
	vector<Vec3f>circles;
	HoughCircles(hough, circles, HOUGH_GRADIENT, 1.13, 1000, 100, 180, 100, 450);

	//cout << circles.size();

	return circles;
}

//获取圆心
cv::Point highPreciseDetector::getCenter() {
	std::vector<cv::Vec3f>circles = this->getCircles();//得到检测得出的圆
	Point center=Point(cvRound(circles[0][0]), cvRound(circles[0][1]));//定义圆心
	return center;//返回
}

//获取半径
double highPreciseDetector::getRadius() {
	std::vector<cv::Vec3f>circles = this->getCircles(); //得到检测出的圆
	double Radius = cvRound(circles[0][2]);//定义半径
	return Radius;//返回
}

//绘制圆
cv::Mat highPreciseDetector::showCircle() {
	//得到圆心和半径
	Mat a = this->img;
	Point center = this->getCenter();
	int r = this->getRadius();
	//绘制
	circle(a, center, 3, Scalar(0, 0, 255), 4);//draw the center
	circle(a, center, r, Scalar(0, 255, 255), 5);//draw the circle

	return a;
}

//得到存储有检测出的指针线段的容器
vector<Vec4i> highPreciseDetector::getLine() {
	Mat a = img;
	//将图像转化为灰度单通道图
	Mat gray;
	cvtColor(a, gray, COLOR_BGR2GRAY);
	//双边滤波
	Mat bf;
	bilateralFilter(gray, bf, LINE_KERNAL, LINE_KERNAL * 2, LINE_KERNAL / 2);
	//二值化处理 
	Mat thre;
	threshold(bf, thre, 130, 400, THRESH_BINARY_INV);
	//边缘检测
	Mat canny;
	Canny(thre, canny, 15, 80, 5);
	//双边滤波
	Mat houghline;
	bilateralFilter(canny, houghline, LINE_KERNAL, LINE_KERNAL * 2, LINE_KERNAL / 2);

	vector<Vec4i>ls;//存储首次检测到的所有线段

	//定义HoughLinesP()参数
	double rho = 0.8;
	double threhold = 40;
	double minLineLength = 50;
	double maxLineGap = 12;
	//霍夫线检测
	HoughLinesP(canny, ls, rho, CV_PI / 180, threhold, minLineLength, maxLineGap);

	//定义合法线段
	vector<Vec4i>legal_lines;

	//去除包含圆外点的不合法线段
	Point c = this->getCenter(); /*std::cout << "c.x" << c.x << " " << "c.y" << c.y << endl;*/
	double r = this->getRadius();/* std::cout << "r" << r << endl;*/
	Point center = this->getCenter();
	//对每个线段的两点进行检测
	for (int i = 0; i < ls.size(); ++i) {
		int x1 = ls[i][0]; int x2 = ls[i][2]; /*cout << "x1 :" << x1 << "    x2:" << x2 << endl;*/
		int y1 = ls[i][1]; int y2 = ls[i][3]; /*cout << "y1 :" << y1 << "    y2:" << y2 << endl;*/
		//以下参数前有一个乘积因子的原因是：
		//在调试的过程中 为避免出现个例不符合预期结果的情况
		//适当更改了参数范围 以保证所有图片均能保留预期结果
		if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 1.1 * pow(r, 2))//如果第一个点在圆内
			if (pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 1.1 * pow(r, 2))//第二个点在圆内
				if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 0.09 * pow(r, 2) ||
					pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 0.09 * pow(r, 2))//如果起始点距离圆心较近
				{
					//cout << "x1 :" << x1 << "    x2:" << x2 << "y1 :" << y1 << "    y2:" << y2 << endl;
					legal_lines.emplace_back(x1, y1, x2, y2);//设置为有效线段
				}
	}

	//调试用：如果合法线段为空，抛出异常
	if (legal_lines.empty()) cerr << "invalid line!" << endl;

	//定义映射 用于找出合法线段的两端点中远距点距离圆心最长的线 该线即为指针线
	unordered_map<double, Vec4i>Maxdistanc_Line;//映射的key存放距离 value存放线段
	for (int i = 0; i < legal_lines.size(); ++i) {
		double dis1 = pow((legal_lines[i][0] - center.x), 2) + pow((legal_lines[i][1] - center.y), 2);//计算端点1与圆心的距离
		double dis2 = pow((legal_lines[i][2] - center.x), 2) + pow((legal_lines[i][3] - center.y), 2);//计算端点2与圆心的距离
		Maxdistanc_Line.emplace((dis1 > dis2 ? dis1 : dis2), legal_lines[i]);//key存放较远距离 value存放线段
	}
	//定义指针线
	Vec4i LonggestLine; double maxdistance = 0;
	//遍历映射
	for (auto& c : Maxdistanc_Line) {
		if (c.first > maxdistance) maxdistance = c.first;//如果距离大于最大距离，那么该距离设置为最大距离
	}
	//通过最大距离key 映射到对应的线段value
	LonggestLine = Maxdistanc_Line[maxdistance];
	
	//最终结果 
	vector<Vec4i>final = { LonggestLine };
	/*std::cout << final.size() << endl;*/
	return final;
}


//得到指针落点位置
cv::Point highPreciseDetector::getFinalPoint() {
	//得到theline  theline即为仪表指针线
	Mat a = img;
	vector<Vec4i>Line = this->getLine();
	Point center = this->getCenter();
	Vec4i theline = Line[0];

	//获取指针线中距离圆心较远的点 即为指针落点
	Point pointerDestination;
	double dis1 = pow((theline[0] - center.x), 2) + pow((theline[1] - center.y), 2);
	double dis2 = pow((theline[2] - center.x), 2) + pow((theline[3] - center.y), 2);
	pointerDestination = dis1 > dis2 ? Point(theline[0], theline[1]) : Point(theline[2], theline[3]);
	
	return pointerDestination;
}

//绘制指针线
cv::Mat highPreciseDetector::showLine() {
	Mat a = img;
	Point center = this->getCenter();
	Point pointerDestination = this->getFinalPoint();
	//首先得到圆心和落点，然后以圆心和落点为参数绘制出指针线
	line(a, center, pointerDestination, Scalar(0, 255, 255), 2);
	return a;
}

//计算角度
//每个指针都有一个角度 
//该角度是指针线与常规坐标系（即x靠右为正 y靠上为正）的横坐标轴的负半轴的夹角
//具体角度的定义可参看实验报告中的解释
double highPreciseDetector::getAngle() {
	//初始化：得到落点和圆心 初始角度为0
	Point pt =this->getFinalPoint();
	Point center = getCenter();
	double angle = 0;

	//将图像坐标系与常规坐标系进行转换
	//以下象限指的都是常规坐标系的一二三四象限
	//先通过反切函数计算夹角 然后进行弧度->角度的转换
	//当角度与常规负半轴重合时 角度为0 顺时针旋转角度增加 逆时针角度减少
	if (pt.x < center.x)
		if (pt.y < center.y) {//第二象限
			double ylen = center.y - pt.y;
			double xlen = center.x - pt.x;
			double arctan = atan(ylen / xlen);
			angle = arctan * 180.0 / pi;
		}
		else {//第三象限
			double ylen = pt.y - center.y;
			double xlen = center.x - pt.x;
			double arctan = atan(ylen / xlen);
			angle = -arctan * 180.0 / pi;
		}
	else {
		if (pt.y < center.y) {//第一象限
			double ylen = center.y - pt.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle = 180.0 - arctan * 180.0 / pi;
		}
		else {//第四象限
			double ylen = pt.y - center.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle = 180.0 + arctan * 180.0 / pi;
		}
	}

	return angle;
}

//计算结果
double highPreciseDetector::getScale1Result() {
	
	double angle = this->getAngle(); //得到指针角
	//cout << angle << endl;
	double calibrationAngle = -51.3977;//定义标定角 标定角的值是调试过程中选择较准确的指针线的角度
	//结果=标定角读数+分度*该角与标定角之差
	double result = 0.4000 + scale1 * (angle - calibrationAngle);

	//设置上下限 防止出现异常读数
	if (result < 0)result = 0.0000;
	if (result > 4)result = 4.0000;
	return result;
}

double highPreciseDetector::getScale2Result() {

	double angle = this->getAngle(); //得到指针角
	//cout << angle << endl;
	double calibrationAngle = -53.5387;//定义标定角 标定角的值是调试过程中选择较准确的指针线的角度
	//结果=标定角读数+分度*该角与标定角之差
	double result = 0.0400 + scale2 * (angle - calibrationAngle);

	//设置上下限 防止出现异常读数
	if (result < 0)result = 0.0000;
	if (result > 4)result = 4.0000;
	return result;
}
//利用putText绘制结果
void highPreciseDetector::showScale1Result() {
	this->showLine();

	this->showCircle();
	
	//定义数字和前缀
	string nums;
	string prefix = "Result:";
	
	double result = this->getScale1Result();//得到结果
	nums = to_string(result);//将结果转化为字符串

	string ans = prefix.append(nums);//最终的字符串

	//绘制
	putText(img, ans, Point(500, 80), FONT_HERSHEY_DUPLEX, 2, Scalar(255, 255, 0), 8, LineTypes::LINE_AA);//write text
	//cv::putText(Frame, "hello", cv::Point(100, 200), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 222, 222), 2);
}

void highPreciseDetector::showScale2Result() {
	this->showLine();

	this->showCircle();

	//定义数字和前缀
	string nums;
	string prefix = "Result:";

	double result = this->getScale2Result();//得到结果
	nums = to_string(result);//将结果转化为字符串

	string ans = prefix.append(nums);//最终的字符串

	//绘制
	putText(img, ans, Point(500, 80), FONT_HERSHEY_DUPLEX, 2, Scalar(255, 255, 0), 8, LineTypes::LINE_AA);//write text
	//cv::putText(Frame, "hello", cv::Point(100, 200), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 222, 222), 2);
}


