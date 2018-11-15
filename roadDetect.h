#pragma once

#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream> 

using namespace cv;
using namespace std;


typedef struct {
	bool flag;             //像素是否被访问
	int pixelValue;        //像素值
}isVisit;

typedef struct {
	Point startPoint;      //连通域起始点
	int regionID;          //连通域标号
	int pointNum;          //连通域像素点数
}connectRegionNumSet;



// grayImg将会返回几个连通域
int calConnectRegionNums(Mat &grayImg, vector<vector<isVisit>>& validImg, vector<connectRegionNumSet> &regionSet, int &grayImgValue);

// 二值化
void Binary(Mat img);

// 孔洞填充
void fillHole(Mat img, Mat &dst);

// 连点
vector<Point> findBoundaryPoint(Mat mask, int grayValue);

void roadRegion(Mat img, vector<Point> pointSet);
