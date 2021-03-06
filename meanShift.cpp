// imageSegment.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "roadDetect.h"
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream> 
#include <time.h>

using namespace cv;
using namespace std; 





int main()
{ 	
	// 桥：CH15-20170515134500.avi
	// 复杂路：E:/video/CH7-20170515141500.avi   CH12-20170515141500.avi
	// 雨天：test.avi

	Mat src;
	string fvideo = "E:/video/CH15-20170515134500.avi";
	VideoCapture cap(fvideo);
	cap >> src;




	// 自动模块
	vector<Point> stack;
	stack.push_back(Point(549, 383));
	stack.push_back(Point(640, 498));
	stack.push_back(Point(646, 491));
	stack.push_back(Point(752, 476));
	stack.push_back(Point(717, 272));
	stack.push_back(Point(1104, 435));
	stack.push_back(Point(1262, 536));
	stack.push_back(Point(1865, 799));


	clock_t startTime = clock();

	roadRegion(src, stack);

	clock_t endTime = clock();
	cout << "Total time:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s\n";

	waitKey(0);
	return 0;
}




