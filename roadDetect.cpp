#include "stdafx.h"
#include "roadDetect.h"

void Binary(Mat img)
{

	for (int i = 0; i < img.rows; i++)
	{
		uchar* data = img.ptr<uchar>(i);
		for (int j = 0; j < img.cols; j++)
		{
			if (data[j] == 0 || data[j] == 255)
				continue;
			else if (data[j] < 10)
				data[i] = 0;
			else
				data[j] = 255;
		}
	}
}

void fillHole(Mat img, Mat &dst)
{
	Binary(img);
	Size size = img.size();
	Mat tempImg(size.height + 2, size.width + 2, CV_8UC1, Scalar::all(0));

	img.copyTo(tempImg(Range(1, size.height + 1), Range(1, size.width + 1)));

	floodFill(tempImg, Point(0, 0), Scalar(255));
	Mat cutImg;
	tempImg(Range(1, size.height + 1), Range(1, size.width + 1)).copyTo(cutImg);

	Mat reverseImg = ~cutImg;

	dst = img | reverseImg;
}

int calConnectRegionNums(Mat &grayImg, vector<vector<isVisit>>& validImg, vector<connectRegionNumSet> &regionSet, int &grayImgValue) {

	int regionID = 1;
	connectRegionNumSet regionSetTemp;  //临时用到的regionSetTemp类型中间变量
										//Mat dst(grayImg.rows, grayImg.cols, CV_8UC1, Scalar::all(0));

	for (int y = 0; y < grayImg.rows; y++)
	{
		uchar *data = grayImg.ptr<uchar>(y);
		//uchar *dataDst = dst.ptr<uchar>(y);
		for (int x = 0; x < grayImg.cols; x++)
		{
			//dataDst[x] = (int)data[x];
			validImg[y][x].pixelValue = (int)data[x];
			validImg[y][x].flag = false;   //开始时默认都未访问

		}
	}

	vector<Point> stack; //stack栈  heap堆
	Point foundValidPoint;

	for (int y = 0; y < grayImg.rows; y++)
	{
		uchar *data = grayImg.ptr<uchar>(y);
		for (int x = 0; x < grayImg.cols; x++)
		{
			if (validImg[y][x].pixelValue && !validImg[y][x].flag)
			{

				data[x] = grayImgValue;

				int eachRegionPixelNum = 1;
				regionSetTemp.startPoint = Point(x, y);  //x表示列，y表示行
				regionSetTemp.regionID = regionID++;
				regionSetTemp.pointNum = 1;

				regionSet.push_back(regionSetTemp);

				validImg[y][x].flag = true;

				stack.push_back(Point(x, y));

				while (stack.size())
				{
					foundValidPoint = stack.back();   //从栈尾开始寻找八连通的相邻点
					stack.pop_back();                 //上一句已得到栈尾像素点，该点可以出栈了

					int i = foundValidPoint.x;
					int j = foundValidPoint.y;

					// find 8-point around foundValidPoint
					int minX = (i - 1 < 0 ? 0 : i - 1);
					int maxX = (i + 1 < grayImg.cols - 1 ? i + 1 : grayImg.cols - 1);
					int minY = (j - 1 < 0 ? 0 : j - 1);
					int maxY = (j + 1 < grayImg.rows - 1 ? j + 1 : grayImg.rows - 1);

					for (int k = minY; k <= maxY; k++)
					{
						for (int t = minX; t <= maxX; t++)
						{
							if (validImg[k][t].pixelValue && !validImg[k][t].flag)
							{
								validImg[k][t].flag = true;
								stack.push_back(Point(t, k));
								eachRegionPixelNum++;
								grayImg.at<uchar>(k, t) = grayImgValue;
							}
						}
					}
				}

				grayImgValue = grayImgValue + 50;

				if (eachRegionPixelNum > 1)
					regionSet[regionSet.size() - 1].pointNum = eachRegionPixelNum;
				else
				{
					regionSet.pop_back();
					regionID--;
				}


			}
		}
	}



	return regionSet.size();

}

vector<Point> findBoundaryPoint(Mat mask, int grayValue)
{
	vector<Point> pointSet;

	for (int k = 50; k <= grayValue; k += 50)
	{
		int minHightX = mask.rows; //
		int minHightY = 0;
		int minWidthX = 0;
		int minWidthY = mask.cols; //
		int maxHightX = 0;         //
		int maxHightY = 0;
		int maxWidthX = 0;
		int maxWidthY = 0;         //
		for (int i = 0; i < mask.rows; i++)
		{
			for (int j = 0; j < mask.cols; j++)
			{
				if (mask.at<uchar>(i, j) == k)
				{
					if (i < minHightX)
					{
						minHightX = i;
						minHightY = j;
					}

					if (j < minWidthY)
					{
						minWidthY = j;
						minWidthX = i;
					}
					if (i > maxHightX)
					{
						maxHightX = i;
						maxHightY = j;
					}
					if (j > maxWidthY)
					{
						maxWidthY = j;
						maxWidthX = i;
					}
				}
			}
		}

		pointSet.push_back(Point(minHightY, minHightX));
		pointSet.push_back(Point(minWidthY, minWidthX));
		pointSet.push_back(Point(maxHightY, maxHightX));
		pointSet.push_back(Point(maxWidthY, maxWidthX));

		line(mask, Point(minHightY, minHightX), Point(minWidthY, minWidthX), Scalar(255, 255, 255), 1, CV_AA);
		line(mask, Point(minWidthY, minWidthX), Point(maxHightY, maxHightX), Scalar(255, 255, 255), 1, CV_AA);
		line(mask, Point(maxHightY, maxHightX), Point(maxWidthY, maxWidthX), Scalar(255, 255, 255), 1, CV_AA);
		line(mask, Point(maxWidthY, maxWidthX), Point(minHightY, minHightX), Scalar(255, 255, 255), 1, CV_AA);
	}

	return pointSet;
}


void roadRegion(Mat img, vector<Point> pointSet)
{
	GaussianBlur(img, img, Size(3, 3), 1);


	Mat dst;
	int spatialRad = 15;
	int colorRad = 30;
	int maxPyrLevel = 1;

	clock_t startTime = clock();

	pyrMeanShiftFiltering(img, dst, spatialRad, colorRad, maxPyrLevel);

	clock_t endTime = clock();
	cout << "meanshift Time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	int lowValue = 2;
	int upValue = 20;
	bool isColor = true;

	Scalar lowdifference = isColor ? Scalar(lowValue, lowValue, lowValue) : Scalar(lowValue);
	Scalar updifference = isColor ? Scalar(upValue, upValue, upValue) : Scalar(upValue);



	startTime = clock();


	while (pointSet.size())
	{
		Point seed = pointSet.back();
		floodFill(dst, seed, Scalar(0, 0, 255), 0, lowdifference, updifference);
		pointSet.pop_back();
	}

	endTime = clock();
	cout << "FloodFill time:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s\n";

	Mat mask(dst.rows, dst.cols, CV_8UC1, Scalar::all(0));

	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			if ((int)dst.at<Vec3b>(i, j)[0] == 0 && (int)dst.at<Vec3b>(i, j)[1] == 0 && (int)dst.at<Vec3b>(i, j)[2] == 255)
			{
				mask.at<uchar>(i, j) = 255;
			}
		}
	}

	fillHole(mask, mask);
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	cv::dilate(mask, mask, element);
	fillHole(mask, mask);


	vector<vector<isVisit>> validImg;

	validImg.resize(mask.rows);

	for (int i = 0; i < validImg.size(); i++)
		validImg[i].resize(mask.cols);

	vector<connectRegionNumSet> regionSet;


	int grayValue = 50;

	startTime = clock();

	int regionNum = calConnectRegionNums(mask, validImg, regionSet, grayValue);

	endTime = clock();
	cout << "ConnectRegion time:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s\n";


	startTime = clock();

	vector<Point> boundaryPoint = findBoundaryPoint(mask, grayValue);

	endTime = clock();
	cout << "Find BoundaryPoint time:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s\n";

	//int nBoundaryPoint = boundaryPoint.size();

	//Point firstPoint = boundaryPoint.back();
	//Point startPoint;
	//Point endPoint;

	//for (int i = 1; i < nBoundaryPoint; i++)
	//{
	//	if (i % 4 == 0)
	//	{
	//		cv::line(mask, endPoint, firstPoint, Scalar(255), 1);

	//		boundaryPoint.pop_back();
	//		firstPoint = boundaryPoint.back();
	//		continue;
	//	}
	//	startPoint = boundaryPoint.back();
	//	boundaryPoint.pop_back();
	//	endPoint = boundaryPoint.back();
	//	cv::line(mask, startPoint, endPoint, Scalar(255), 1);
	//}




	cv::imshow("meanshift", dst);
	cv::imshow("mask", mask);
}