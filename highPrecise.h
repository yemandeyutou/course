#pragma once

//此文件用于检测高精度的四张图片
#ifndef HIGHPRECISE
#define HIGHPRECISE

//本项目采用opencv4.7.0 windows 使用的接口可兼容多版本
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include<vector>

class highPreciseDetector {//高精度检测器类
public:
	highPreciseDetector() = delete;//默认构造函数被删除
	highPreciseDetector(cv::Mat img) { this->img = img; }//初始化

	void showScale1Result();//用于显示最终结果  检测器对象可调用该函数
	void showScale2Result();//用于显示最终结果  检测器对象可调用该函数

protected://protected 函数仅用于类内部获取数据、计算、实现结果
	cv::Mat showLine();//画出指针线
	cv::Mat showCircle();//画出仪表盘
	std::vector<cv::Vec4i> getLine();//得到存储有指针线段的容器
	double getAngle();//计算偏转角度并返回
	cv::Point getFinalPoint();//得到指针落点位置
	std::vector<cv::Vec3f>getCircles();//得到存储有仪表盘圆的容器
	cv::Point getCenter();//获取圆心
	double getRadius();//获取半径
	double getScale1Result();
	double getScale2Result();


private:
	cv::Mat img;//存储读入的图像
};
//具体实现请参看highPrecise.cpp

#endif // !HIGHPRECISE

