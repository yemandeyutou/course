#pragma once

//���ļ����ڼ��;��ȵĶ�ʮ����ͼƬ
#ifndef  LOWPRECISE
#define  LOWPRECISE

//����Ŀ����opencv4.7.0 windows ʹ�õĽӿڿɼ��ݶ�汾
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include<vector>


class lowPreciseDetector {//�;��ȼ����
public:
	lowPreciseDetector() = delete; //Ĭ�Ϲ��캯����ɾ�� 
	lowPreciseDetector(cv::Mat  img) { this->img = img; }//��ʼ��
	
	void showResult();//������ʾ���ս��  ���������ɵ��øú���

protected://protected �������������ڲ���ȡ���ݡ����㡢ʵ�ֽ��
	std::vector<cv::Vec3f> getCircles();//�õ��洢�Ǳ���Բ������
	cv::Point getCenter();//��ȡԲ��
	double getRadius();//��ȡ�뾶
	cv::Mat showCircle();//������⵽��Բ
	std::vector<cv::Vec4i> getLine();//��ȡ�洢ָ���߶ε�����
	cv::Mat showLine();//����������ָ��
	cv::Point getFinalPoint();//��ȡָ������
	double getAngle();//����ƫת�ǶȲ�����
	double getResult();//������������
private:
	cv::Mat img;//�洢��ȡ��ͼ��
};
//����ʵ����ο�lowPrecise.cpp

#endif // ! LOWPRECISE


