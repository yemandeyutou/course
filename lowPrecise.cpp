#include<math.h>
#include<iostream>
#include<unordered_map>
#include<algorithm>
#include"lowPrecise.h"

using namespace std;
using namespace cv;
//�;��ȼ�������ʵ��

//�궨�� ���ڼ���ָ��̶� ���㹫ʽΪ���̶�=ֵ/�Ƕ�
const double scale = (0.5251 - 0.06842)/(197.819-(-10.4207)) ;
//�����
const double pi = 3.141592654;

//�����߼��ʱ ʹ�õĺ˴�С=
const int LINEKERNAL = 13;

//�õ��洢���Ǳ���Բ������
vector<Vec3f> lowPreciseDetector::getCircles() {
	//��ͼ��ת��Ϊ�Ҷȵ�ͨ��ͼ�� 
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	
	medianBlur(gray, gray, 3);//�ԻҶȺ��ͼ�������ֵģ��
	vector<Vec3f>circles;

	//��HoughCircle()��⵽��Բ�洢��circles������
	HoughCircles(gray, circles, HOUGH_GRADIENT, 3, 30, 180, 300, 100, 130);
	return circles;
}

//��ȡԲ��
Point lowPreciseDetector::getCenter() {
	vector<Vec3f>circles;
	circles = this->getCircles();//�õ����ó���Բ
	Point center = Point(cvRound(circles[0][0]), cvRound(circles[0][1]));//����Բ��
	return center;
}
//��ȡ�뾶
double lowPreciseDetector::getRadius() {
	vector<Vec3f>circles;
	circles = this->getCircles();//�õ�������Բ
	double Radius = cvRound(circles[0][2]);//����뾶
	return Radius;
}

//����Բ
cv::Mat lowPreciseDetector::showCircle() {
	/*Vec3f circle = this->getCircles()[0];*/
	Mat a = img;
	Point center = this->getCenter();//�õ�Բ��
	int Radius = this->getRadius();//�õ��뾶
	//����
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

//��ȡ�洢ָ���ߵ�����
vector<Vec4i> lowPreciseDetector::getLine() {
	//ͼ��ת��Ϊ�Ҷȵ�ͨ��
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	//˫���˲�
	Mat bf;
	bilateralFilter(gray, bf, LINEKERNAL, LINEKERNAL * 2, LINEKERNAL / 2);
	//��ֵ������
	Mat thre;
	threshold(bf, thre, 130, 400, THRESH_BINARY);
	//canny��Ե���
	Mat canny;
	Canny(thre, canny, 15, 80, 5);
	//˫���˲�
	Mat houghline;
	bilateralFilter(canny, houghline, LINEKERNAL, LINEKERNAL * 2, LINEKERNAL / 2);

	//����houghlinep()��⵽���߶κͲ���
	vector<Vec4i>Lines;
	double rho = 1.11;
	double threhold = 40;
	double minLineLength = 20;
	double maxLineGap = 12;
	//�����߼��
	HoughLinesP(houghline, Lines, rho, CV_PI / 180, threhold, minLineLength, maxLineGap);

	//������Ч�߶�
	vector<Vec4i>valid_Lines;
	//��ȡԲ�ĺͰ뾶
	Point center = getCenter();
	double radius = this->getRadius();
	//�����������߶� ɸѡ����Ч�߶�
	//��ÿ���߶ε�������м��
	for (int i = 0; i < Lines.size(); ++i) {
		int x1 = Lines[i][0]; int x2 = Lines[i][2]; /*cout << "x1 :" << x1 << "    x2:" << x2 << endl;*/
		int y1 = Lines[i][1]; int y2 = Lines[i][3]; /*cout << "y1 :" << y1 << "    y2:" << y2 << endl;*/
		//���²���ǰ��һ���˻����ӵ�ԭ���ǣ�
		//�ڵ��ԵĹ����� Ϊ������ָ���������Ԥ�ڽ�������
		//�ʵ������˲�����Χ �Ա�֤����ͼƬ���ܱ���Ԥ�ڽ��
		if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 1.1 * pow(radius, 2))//�����һ������Բ��
			if (pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 1.1 * pow(radius, 2))//�ڶ�������Բ��
				if (pow((x1 - center.x), 2) + pow((y1 - center.y), 2) < 0.09 * pow(radius, 2) ||
					pow((x2 - center.x), 2) + pow((y2 - center.y), 2) < 0.09 * pow(radius, 2))//�����ʼ�����Բ�ĽϽ�
				{
					valid_Lines.emplace_back(x1, y1, x2, y2);//����Ϊ��Ч�߶�
				}
	}
	//�����ã�����Ϸ��߶�Ϊ�գ��׳��쳣
	if (valid_Lines.empty()) cerr << "invalid line!" << endl;

	//����ӳ�� �����ҳ��Ϸ��߶ε����˵���Զ������Բ������� ���߼�Ϊָ����
	unordered_map<double, Vec4i>Maxdistanc_Line;//ӳ���key��ž��� value����߶�
	for (int i = 0; i < valid_Lines.size(); ++i) {
		double dis1 = pow((valid_Lines[i][0] - center.x), 2) + pow((valid_Lines[i][1] - center.y), 2);//����˵�1��Բ�ĵľ���
		double dis2 = pow((valid_Lines[i][2] - center.x), 2) + pow((valid_Lines[i][3] - center.y), 2);//����˵�2��Բ�ĵľ���
		Maxdistanc_Line.emplace((dis1 > dis2 ? dis1 : dis2), valid_Lines[i]);//key��Ž�Զ���� value����߶�
	}
	//����ָ����
	Vec4i LonggestLine; double maxdistance = 0;
	//����ӳ��
	for (auto& c : Maxdistanc_Line) {
		if (c.first > maxdistance) maxdistance = c.first;
	}
	//ͨ��������key ӳ�䵽��Ӧ���߶�value
	LonggestLine = Maxdistanc_Line[maxdistance];

	//���ս�� 
	vector<Vec4i>final = { LonggestLine };
	/*std::cout << final.size() << endl;*/
	return final;
}

//����ָ����
Mat lowPreciseDetector::showLine() {
	Mat a = img;
	Point center = this->getCenter();
	Point pointerDestination=getFinalPoint();
	//���ȵõ�Բ�ĺ���㣬Ȼ����Բ�ĺ����Ϊ�������Ƴ�ָ����
	line(a, center, pointerDestination ,Scalar(0, 255, 255), 2);

	return a;
}


//�õ�ָ�����λ��
cv::Point lowPreciseDetector::getFinalPoint() {
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

//����Ƕ�
//ÿ��ָ�붼��һ���Ƕ� 
//�ýǶ���ָ�����볣������ϵ����x����Ϊ�� y����Ϊ�����ĺ�������ĸ�����ļн�
//����ǶȵĶ���ɲο�ʵ�鱨���еĽ���
double lowPreciseDetector::getAngle() {
	//��ʼ�����õ�����Բ�� ��ʼ�Ƕ�Ϊ0
	Point pt = getFinalPoint();
	Point center = getCenter();
	double angle=0;

	//��ͼ������ϵ�볣������ϵ����ת��
	//��������ָ�Ķ��ǳ�������ϵ��һ����������
	//��ͨ�����к�������н� Ȼ����л���->�Ƕȵ�ת��
	//���Ƕ��볣�渺�����غ�ʱ �Ƕ�Ϊ0 ˳ʱ����ת�Ƕ����� ��ʱ��Ƕȼ���
	if(pt.x<center.x)
		if (pt.y < center.y) {//�ڶ�����
			double ylen = center.y-pt.y;
			double xlen = center.x-pt.x;
			double arctan = atan(ylen / xlen);
			angle = arctan * 180.0 / pi;
		}
		else {//��������
			double ylen = pt.y- center.y;
			double xlen = center.x - pt.x;
			double arctan = atan(ylen / xlen);
			angle = -arctan * 180.0/ pi;
		}
	else {
		if (pt.y < center.y) {//��һ����
			double ylen = center.y - pt.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle =180.0-arctan * 180.0 / pi;
		}
		else {//��������
			double ylen = pt.y-center.y;
			double xlen = pt.x - center.x;
			double arctan = atan(ylen / xlen);
			angle = 180.0+arctan * 180.0 / pi;
		}
	}
	return angle;
}

//������
double lowPreciseDetector::getResult() {

	double angle = this->getAngle(); /*cout << "angle: " << angle << endl;*/ //�õ�ָ���
	double calibrationAngle = -9.78241;//����궨�� �궨�ǵ�ֵ�ǵ��Թ�����ѡ���׼ȷ��ָ���ߵĽǶ�

	//���=�궨�Ƕ���+�ֶ�*�ý���궨��֮��
	double result = 0.06842+scale*(angle-calibrationAngle);
	//���������� ��ֹ�����쳣����
	if (result < 0)result = 0.000000;
	if (result > 0.6)result = 0.600000;
	return result;
}

//����putText���ƽ��
void lowPreciseDetector::showResult() {
	this->showCircle();
	this->showLine();
	//�������ֺ�ǰ׺
	string nums;
	string prefix="Result:";
	double result = this->getResult(); //�õ����
	nums = to_string(result);//�����ת��Ϊ�ַ���
	//���յ��ַ���
	string ans = prefix.append(nums);

	//����
	putText(img, ans, Point(50, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 0), 1,LineTypes::LINE_AA);
}