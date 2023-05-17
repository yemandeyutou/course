#pragma once

//���ļ����ڼ��߾��ȵ�����ͼƬ
#ifndef HIGHPRECISE
#define HIGHPRECISE

//����Ŀ����opencv4.7.0 windows ʹ�õĽӿڿɼ��ݶ�汾
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include<vector>

class highPreciseDetector {//�߾��ȼ������
public:
	highPreciseDetector() = delete;//Ĭ�Ϲ��캯����ɾ��
	highPreciseDetector(cv::Mat img) { this->img = img; }//��ʼ��

	void showScale1Result();//������ʾ���ս��  ���������ɵ��øú���
	void showScale2Result();//������ʾ���ս��  ���������ɵ��øú���

protected://protected �������������ڲ���ȡ���ݡ����㡢ʵ�ֽ��
	cv::Mat showLine();//����ָ����
	cv::Mat showCircle();//�����Ǳ���
	std::vector<cv::Vec4i> getLine();//�õ��洢��ָ���߶ε�����
	double getAngle();//����ƫת�ǶȲ�����
	cv::Point getFinalPoint();//�õ�ָ�����λ��
	std::vector<cv::Vec3f>getCircles();//�õ��洢���Ǳ���Բ������
	cv::Point getCenter();//��ȡԲ��
	double getRadius();//��ȡ�뾶
	double getScale1Result();
	double getScale2Result();


private:
	cv::Mat img;//�洢�����ͼ��
};
//����ʵ����ο�highPrecise.cpp

#endif // !HIGHPRECISE

