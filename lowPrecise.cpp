#include<math.h>
#include<iostream>
#include<unordered_map>
#include<algorithm>
#include"lowPrecise.h"

using namespace std;
using namespace cv;
//低精度检测器类的实现

//标定用 用于计算指针刻度 计算公式为：刻度=值/角度
const double scale = (0.5251 - 0.06842)/(197.819-(-10.4207)) ;
//定义π
const double pi = 3.141592654;

//定义线检测时 使用的核大小=
const int LINEKERNAL = 13;

//得到存储有仪表盘圆的容器
vector<Vec3f> lowPreciseDetector::getCircles() {
	//将图像转换为灰度单通道图像 
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	
	medianBlur(gray, gray, 3);//对灰度后的图像进行中值模糊
	vector<Vec3f>circles;

	//将HoughCircle()检测到的圆存储到circles容器中
	HoughCircles(gray, circles, HOUGH_GRADIENT, 3, 30, 180, 300, 100, 130);
	return circles;
}

//获取圆心
Point lowPreciseDetector::getCenter() {
	vector<Vec3f>circles;
	circles = this->getCircles();//得到检测得出的圆
	Point center = Point(cvRound(circles[0][0]), cvRound(circles[0][1]));//定义圆心
	return center;
}
//获取半径
double lowPreciseDetector::getRadius() {
	vector<Vec3f>circles;
	circles = this->getCircles();//得到检测出的圆
	double Radius = cvRound(circles[0][2]);//定义半径
	return Radius;
}

//绘制圆
cv::Mat lowPreciseDetector::showCircle() {
	/*Vec3f circle = this->getCircles()[0];*/
	Mat a = img;
	Point center = this->getCenter();//得到圆心
	int Radius = this->getRadius();//得到半径
	//绘制
	cv::circle(a, center, 1, Scalar(0, 0, 255), 4);//draw the center 
	cv::circle(a, center, Radius, Scalar(0, 255, 255), 2);//draw the circle
	//for (int i = 0; i < circles.size(); ++i) {
	//	Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	//	int Radius = cvRound(circles[i][2]);
	//	circle(a, center, 1, Scalar(0, 0, 255), 2);//draw the center
	//	circle(a, center, Radius, Scalar(0, 255, 255), 2);//draw the circle
	//}
	return a;
}

//获取存储指针线的容器
vector<Vec4i> lowPreciseDetector::getLine() {
	//图像转化为灰度单通道
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	//双边滤波
	Mat bf;
	bilateralFilter(gray, bf, LINEKERNAL, LINEKERNAL * 2, LINEKERNAL / 2);
	//二值化处理
	Mat thre;
	threshold(bf, thre, 130, 400, THRESH_BINARY);
	//canny边缘检测
	Mat canny;
	Canny(thre, canny, 15, 80, 5);
	//双边滤波
	Mat houghline;
	bilateralFilter(canny, houghline, LINEKERNAL, LINEKERNAL * 2, LINEKERNAL / 2);

	//定义houghlinep()检测到的线段和参数
	vector<Vec4i>Lines;
	double rho = 1.11;
	double threhold = 40;
	double minLineLength = 20;
	double maxLineGap = 12;
	//霍夫线检测
	HoughLinesP(houghline, Lines, rho, CV_PI / 180, threhold, minLineLength, maxLineGap);

	//定义有效线段
	vector<Vec4i>valid_Lines;
	//获取圆心和半径
	Point center = getCenter();
	double radius = this->getRadius();
	//遍历检测出的线段 筛选出有效线段
	//对每个线段的两点进行检测
	for (int i = 0; i < Lines.size(); ++i) {
		int x1 = Lines[i][0]; int x2 = Lines[i][2]; /*cout << "x1 :" << x1 << "    x2:" << x2 << endl;*/
		int y1 = Lines[i][1]; int y2 = Lines[i][3]; /*cout << "y1 :" << y1 << "    y2:" << y2 << endl;*/
		//以下参数前有一个乘积因子的原因是：
		//在调试的过程中 为避免出现个例不符合预期结果的情况
		//适当更改了参数范围 以保证所有图片均能保留预期结果
		if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 1.1 * pow(radius, 2))//如果第一个点在圆内
			if (pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 1.1 * pow(radius, 2))//第二个点在圆内
				if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 0.09 * pow(radius, 2) ||
					pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 0.09 * pow(radius, 2))//如果起始点距离圆心较近
				{
					valid_Lines.emplace_back(x1, y1, x2, y2);//设置为有效线段
				}
	}
	//调试用：如果合法线段为空，抛出异常
	if (valid_Lines.empty()) cerr << "invalid line!" << endl;

	//定义映射 用于找出合法线段的两端点中远距点距离圆心最长的线 该线即为指针线
	unordered_map<double, Vec4i>Maxdistanc_Line;//映射的key存放距离 value存放线段
	for (int i = 0; i < valid_Lines.size(); ++i) {
		double dis1 = pow((valid_Lines[i][0] - center.x), 2) + pow((valid_Lines[i][1] - center.y), 2);//计算端点1与圆心的距离
		double dis2 = pow((valid_Lines[i][2] - center.x), 2) + pow((valid_Lines[i][3] - center.y), 2);//计算端点2与圆心的距离
		Maxdistanc_Line.emplace((dis1 > dis2 ? dis1 : dis2), valid_Lines[i]);//key存放较远距离 value存放线段
	}
	//定义指针线
	Vec4i LonggestLine; double maxdistance = 0;
	//遍历映射
	for (auto& c : Maxdistanc_Line) {
		if (c.first > maxdistance) maxdistance = c.first;
	}
	//通过最大距离key 映射到对应的线段value
	LonggestLine = Maxdistanc_Line[maxdistance];

	//最终结果 
	vector<Vec4i>final = { LonggestLine };
	/*std::cout << final.size() << endl;*/
	return final;
}

//绘制指针线
Mat lowPreciseDetector::showLine() {
	Mat a = img;
	Point center = this->getCenter();
	Point pointerDestination=getFinalPoint();
	//首先得到圆心和落点，然后以圆心和落点为参数绘制出指针线
	line(a, center, pointerDestination ,Scalar(0, 255, 255), 2);

	return a;
}


//得到指针落点位置
cv::Point lowPreciseDetector::getFinalPoint() {
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

//计算角度
//每个指针都有一个角度 
//该角度是指针线与常规坐标系（即x靠右为正 y靠上为正）的横坐标轴的负半轴的夹角
//具体角度的定义可参看实验报告中的解释
double lowPreciseDetector::getAngle() {
	//初始化：得到落点和圆心 初始角度为0
	Point pt = getFinalPoint();
	Point center = getCenter();
	double angle=0;

	//将图像坐标系与常规坐标系进行转换
	//以下象限指的都是常规坐标系的一二三四象限
	//先通过反切函数计算夹角 然后进行弧度->角度的转换
	//当角度与常规负半轴重合时 角度为0 顺时针旋转角度增加 逆时针角度减少
	if(pt.x<center.x)
		if (pt.y < center.y) {//第二象限
			double ylen = center.y-pt.y;
			double xlen = center.x-pt.x;
			double arctan = atan(ylen / xlen);
			angle = arctan * 180.0 / pi;
		}
		else {//第三象限
			double ylen = pt.y- center.y;
			double xlen = center.x - pt.x;
			double arctan = atan(ylen / xlen);
			angle = -arctan * 180.0/ pi;
		}
	else {
		if (pt.y < center.y) {//第一象限
			double ylen = center.y - pt.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle =180.0-arctan * 180.0 / pi;
		}
		else {//第四象限
			double ylen = pt.y-center.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle = 180.0+arctan * 180.0 / pi;
		}
	}
	return angle;
}

//计算结果
double lowPreciseDetector::getResult() {

	double angle = this->getAngle(); /*cout << "angle: " << angle << endl;*/ //得到指针角
	double calibrationAngle = -9.78241;//定义标定角 标定角的值是调试过程中选择较准确的指针线的角度

	//结果=标定角读数+分度*该角与标定角之差
	double result = 0.06842+scale*(angle-calibrationAngle);
	//设置上下限 防止出现异常读数
	if (result < 0)result = 0.000000;
	if (result > 0.6)result = 0.600000;
	return result;
}

//利用putText绘制结果
void lowPreciseDetector::showResult() {
	this->showCircle();
	this->showLine();
	//定义数字和前缀
	string nums;
	string prefix="Result:";
	double result = this->getResult(); //得到结果
	nums = to_string(result);//将结果转化为字符串
	//最终的字符串
	string ans = prefix.append(nums);

	//绘制
	putText(img, ans, Point(50, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 0), 1,LineTypes::LINE_AA);
}