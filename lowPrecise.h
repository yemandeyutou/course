#pragma once

//此文件用于检测低精度的二十五张图片
#ifndef  LOWPRECISE
#define  LOWPRECISE

//本项目采用opencv4.7.0 windows 使用的接口可兼容多版本
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include<vector>


class lowPreciseDetector {//低精度检测器
public:
	lowPreciseDetector() = delete; //默认构造函数被删除 
	lowPreciseDetector(cv::Mat  img) { this->img = img; }//初始化
	
	void showResult();//用于显示最终结果  检测器对象可调用该函数

protected://protected 函数仅用于类内部获取数据、计算、实现结果
	std::vector<cv::Vec3f> getCircles();//得到存储仪表盘圆的容器
	cv::Point getCenter();//获取圆心
	double getRadius();//获取半径
	cv::Mat showCircle();//画出检测到的圆
	std::vector<cv::Vec4i> getLine();//获取存储指针线段的容器
	cv::Mat showLine();//画出检测出的指针
	cv::Point getFinalPoint();//获取指针的落点
	double getAngle();//计算偏转角度并返回
	double getResult();//计算结果并返回
private:
	cv::Mat img;//存储获取的图像
};
//具体实现请参看lowPrecise.cpp

#endif // ! LOWPRECISE


