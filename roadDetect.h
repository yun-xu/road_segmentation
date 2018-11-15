#pragma once

#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream> 

using namespace cv;
using namespace std;


typedef struct {
	bool flag;             //�����Ƿ񱻷���
	int pixelValue;        //����ֵ
}isVisit;

typedef struct {
	Point startPoint;      //��ͨ����ʼ��
	int regionID;          //��ͨ����
	int pointNum;          //��ͨ�����ص���
}connectRegionNumSet;



// grayImg���᷵�ؼ�����ͨ��
int calConnectRegionNums(Mat &grayImg, vector<vector<isVisit>>& validImg, vector<connectRegionNumSet> &regionSet, int &grayImgValue);

// ��ֵ��
void Binary(Mat img);

// �׶����
void fillHole(Mat img, Mat &dst);

// ����
vector<Point> findBoundaryPoint(Mat mask, int grayValue);

void roadRegion(Mat img, vector<Point> pointSet);
