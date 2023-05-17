#include"highPrecise.h"
#include<math.h>
#include<iostream>
#include<string>
#include<unordered_map>
#include<algorithm>

using namespace std;
using namespace cv;
//�߾��ȼ�������ʵ��

const double pi = 3.141592654;//�����


//�궨�� ���ڼ���ָ��̶� ���㹫ʽΪ���̶�=ֵ/�Ƕ�
const double scale1 = 0.3999 / (-16.8256 -(-51.3977));//  value/angle  
const double scale2 = 0.3599 / (-30.4572 - (-53.5387));
//�����߼��ʱ ʹ�õĺ˴�С
const int LINE_KERNAL = 9;

//����Բ���ʱ ʹ�õĺ˴�С
const int CIRCLE_KERNALVALUE = 11;


//�õ��洢���Ǳ���Բ������
std::vector<cv::Vec3f> highPreciseDetector::getCircles() {
	
	//��ͼ��ת��Ϊ�Ҷȵ�ͨ��ͼ�� 
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	//˫���˲� ����ȥ������
	Mat bf;
	bilateralFilter(gray, bf, CIRCLE_KERNALVALUE, CIRCLE_KERNALVALUE * 2, CIRCLE_KERNALVALUE / 2);
	//��ȡͼ���Ե
	Mat canny;
	Canny(bf, canny, 10, 250, 5);
	//˫���˲�
	Mat hough;
	bilateralFilter(canny, hough, CIRCLE_KERNALVALUE, CIRCLE_KERNALVALUE * 2, CIRCLE_KERNALVALUE / 2);
	//��HoughCircle()��⵽��Բ�洢��circles������
	vector<Vec3f>circles;
	HoughCircles(hough, circles, HOUGH_GRADIENT, 1.13, 1000, 100, 180, 100, 450);

	//cout << circles.size();

	return circles;
}

//��ȡԲ��
cv::Point highPreciseDetector::getCenter() {
	std::vector<cv::Vec3f>circles = this->getCircles();//�õ����ó���Բ
	Point center=Point(cvRound(circles[0][0]), cvRound(circles[0][1]));//����Բ��
	return center;//����
}

//��ȡ�뾶
double highPreciseDetector::getRadius() {
	std::vector<cv::Vec3f>circles = this->getCircles(); //�õ�������Բ
	double Radius = cvRound(circles[0][2]);//����뾶
	return Radius;//����
}

//����Բ
cv::Mat highPreciseDetector::showCircle() {
	//�õ�Բ�ĺͰ뾶
	Mat a = this->img;
	Point center = this->getCenter();
	int r = this->getRadius();
	//����
	circle(a, center, 3, Scalar(0, 0, 255), 4);//draw the center
	circle(a, center, r, Scalar(0, 255, 255), 5);//draw the circle

	return a;
}

//�õ��洢�м�����ָ���߶ε�����
vector<Vec4i> highPreciseDetector::getLine() {
	Mat a = img;
	//��ͼ��ת��Ϊ�Ҷȵ�ͨ��ͼ
	Mat gray;
	cvtColor(a, gray, COLOR_BGR2GRAY);
	//˫���˲�
	Mat bf;
	bilateralFilter(gray, bf, LINE_KERNAL, LINE_KERNAL * 2, LINE_KERNAL / 2);
	//��ֵ������ 
	Mat thre;
	threshold(bf, thre, 130, 400, THRESH_BINARY_INV);
	//��Ե���
	Mat canny;
	Canny(thre, canny, 15, 80, 5);
	//˫���˲�
	Mat houghline;
	bilateralFilter(canny, houghline, LINE_KERNAL, LINE_KERNAL * 2, LINE_KERNAL / 2);

	vector<Vec4i>ls;//�洢�״μ�⵽�������߶�

	//����HoughLinesP()����
	double rho = 0.8;
	double threhold = 40;
	double minLineLength = 50;
	double maxLineGap = 12;
	//�����߼��
	HoughLinesP(canny, ls, rho, CV_PI / 180, threhold, minLineLength, maxLineGap);

	//����Ϸ��߶�
	vector<Vec4i>legal_lines;

	//ȥ������Բ���Ĳ��Ϸ��߶�
	Point c = this->getCenter(); /*std::cout << "c.x" << c.x << " " << "c.y" << c.y << endl;*/
	double r = this->getRadius();/* std::cout << "r" << r << endl;*/
	Point center = this->getCenter();
	//��ÿ���߶ε�������м��
	for (int i = 0; i < ls.size(); ++i) {
		int x1 = ls[i][0]; int x2 = ls[i][2]; /*cout << "x1 :" << x1 << "    x2:" << x2 << endl;*/
		int y1 = ls[i][1]; int y2 = ls[i][3]; /*cout << "y1 :" << y1 << "    y2:" << y2 << endl;*/
		//���²���ǰ��һ���˻����ӵ�ԭ���ǣ�
		//�ڵ��ԵĹ����� Ϊ������ָ���������Ԥ�ڽ�������
		//�ʵ������˲�����Χ �Ա�֤����ͼƬ���ܱ���Ԥ�ڽ��
		if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 1.1 * pow(r, 2))//�����һ������Բ��
			if (pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 1.1 * pow(r, 2))//�ڶ�������Բ��
				if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 0.09 * pow(r, 2) ||
					pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 0.09 * pow(r, 2))//�����ʼ�����Բ�ĽϽ�
				{
					//cout << "x1 :" << x1 << "    x2:" << x2 << "y1 :" << y1 << "    y2:" << y2 << endl;
					legal_lines.emplace_back(x1, y1, x2, y2);//����Ϊ��Ч�߶�
				}
	}

	//�����ã�����Ϸ��߶�Ϊ�գ��׳��쳣
	if (legal_lines.empty()) cerr << "invalid line!" << endl;

	//����ӳ�� �����ҳ��Ϸ��߶ε����˵���Զ������Բ������� ���߼�Ϊָ����
	unordered_map<double, Vec4i>Maxdistanc_Line;//ӳ���key��ž��� value����߶�
	for (int i = 0; i < legal_lines.size(); ++i) {
		double dis1 = pow((legal_lines[i][0] - center.x), 2) + pow((legal_lines[i][1] - center.y), 2);//����˵�1��Բ�ĵľ���
		double dis2 = pow((legal_lines[i][2] - center.x), 2) + pow((legal_lines[i][3] - center.y), 2);//����˵�2��Բ�ĵľ���
		Maxdistanc_Line.emplace((dis1 > dis2 ? dis1 : dis2), legal_lines[i]);//key��Ž�Զ���� value����߶�
	}
	//����ָ����
	Vec4i LonggestLine; double maxdistance = 0;
	//����ӳ��
	for (auto& c : Maxdistanc_Line) {
		if (c.first > maxdistance) maxdistance = c.first;//���������������룬��ô�þ�������Ϊ������
	}
	//ͨ��������key ӳ�䵽��Ӧ���߶�value
	LonggestLine = Maxdistanc_Line[maxdistance];
	
	//���ս�� 
	vector<Vec4i>final = { LonggestLine };
	/*std::cout << final.size() << endl;*/
	return final;
}


//�õ�ָ�����λ��
cv::Point highPreciseDetector::getFinalPoint() {
	//�õ�theline  theline��Ϊ�Ǳ�ָ����
	Mat a = img;
	vector<Vec4i>Line = this->getLine();
	Point center = this->getCenter();
	Vec4i theline = Line[0];

	//��ȡָ�����о���Բ�Ľ�Զ�ĵ� ��Ϊָ�����
	Point pointerDestination;
	double dis1 = pow((theline[0] - center.x), 2) + pow((theline[1] - center.y), 2);
	double dis2 = pow((theline[2] - center.x), 2) + pow((theline[3] - center.y), 2);
	pointerDestination = dis1 > dis2 ? Point(theline[0], theline[1]) : Point(theline[2], theline[3]);
	
	return pointerDestination;
}

//����ָ����
cv::Mat highPreciseDetector::showLine() {
	Mat a = img;
	Point center = this->getCenter();
	Point pointerDestination = this->getFinalPoint();
	//���ȵõ�Բ�ĺ���㣬Ȼ����Բ�ĺ����Ϊ�������Ƴ�ָ����
	line(a, center, pointerDestination, Scalar(0, 255, 255), 2);
	return a;
}

//����Ƕ�
//ÿ��ָ�붼��һ���Ƕ� 
//�ýǶ���ָ�����볣������ϵ����x����Ϊ�� y����Ϊ�����ĺ�������ĸ�����ļн�
//����ǶȵĶ���ɲο�ʵ�鱨���еĽ���
double highPreciseDetector::getAngle() {
	//��ʼ�����õ�����Բ�� ��ʼ�Ƕ�Ϊ0
	Point pt =this->getFinalPoint();
	Point center = getCenter();
	double angle = 0;

	//��ͼ������ϵ�볣������ϵ����ת��
	//��������ָ�Ķ��ǳ�������ϵ��һ����������
	//��ͨ�����к�������н� Ȼ����л���->�Ƕȵ�ת��
	//���Ƕ��볣�渺�����غ�ʱ �Ƕ�Ϊ0 ˳ʱ����ת�Ƕ����� ��ʱ��Ƕȼ���
	if (pt.x < center.x)
		if (pt.y < center.y) {//�ڶ�����
			double ylen = center.y - pt.y;
			double xlen = center.x - pt.x;
			double arctan = atan(ylen / xlen);
			angle = arctan * 180.0 / pi;
		}
		else {//��������
			double ylen = pt.y - center.y;
			double xlen = center.x - pt.x;
			double arctan = atan(ylen / xlen);
			angle = -arctan * 180.0 / pi;
		}
	else {
		if (pt.y < center.y) {//��һ����
			double ylen = center.y - pt.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle = 180.0 - arctan * 180.0 / pi;
		}
		else {//��������
			double ylen = pt.y - center.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle = 180.0 + arctan * 180.0 / pi;
		}
	}

	return angle;
}

//������
double highPreciseDetector::getScale1Result() {
	
	double angle = this->getAngle(); //�õ�ָ���
	//cout << angle << endl;
	double calibrationAngle = -51.3977;//����궨�� �궨�ǵ�ֵ�ǵ��Թ�����ѡ���׼ȷ��ָ���ߵĽǶ�
	//���=�궨�Ƕ���+�ֶ�*�ý���궨��֮��
	double result = 0.4000 + scale1 * (angle - calibrationAngle);

	//���������� ��ֹ�����쳣����
	if (result < 0)result = 0.0000;
	if (result > 4)result = 4.0000;
	return result;
}

double highPreciseDetector::getScale2Result() {

	double angle = this->getAngle(); //�õ�ָ���
	//cout << angle << endl;
	double calibrationAngle = -53.5387;//����궨�� �궨�ǵ�ֵ�ǵ��Թ�����ѡ���׼ȷ��ָ���ߵĽǶ�
	//���=�궨�Ƕ���+�ֶ�*�ý���궨��֮��
	double result = 0.0400 + scale2 * (angle - calibrationAngle);

	//���������� ��ֹ�����쳣����
	if (result < 0)result = 0.0000;
	if (result > 4)result = 4.0000;
	return result;
}
//����putText���ƽ��
void highPreciseDetector::showScale1Result() {
	this->showLine();

	this->showCircle();
	
	//�������ֺ�ǰ׺
	string nums;
	string prefix = "Result:";
	
	double result = this->getScale1Result();//�õ����
	nums = to_string(result);//�����ת��Ϊ�ַ���

	string ans = prefix.append(nums);//���յ��ַ���

	//����
	putText(img, ans, Point(500, 80), FONT_HERSHEY_DUPLEX, 2, Scalar(255, 255, 0), 8, LineTypes::LINE_AA);//write text
	//cv::putText(Frame, "hello", cv::Point(100, 200), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 222, 222), 2);
}

void highPreciseDetector::showScale2Result() {
	this->showLine();

	this->showCircle();

	//�������ֺ�ǰ׺
	string nums;
	string prefix = "Result:";

	double result = this->getScale2Result();//�õ����
	nums = to_string(result);//�����ת��Ϊ�ַ���

	string ans = prefix.append(nums);//���յ��ַ���

	//����
	putText(img, ans, Point(500, 80), FONT_HERSHEY_DUPLEX, 2, Scalar(255, 255, 0), 8, LineTypes::LINE_AA);//write text
	//cv::putText(Frame, "hello", cv::Point(100, 200), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 222, 222), 2);
}


