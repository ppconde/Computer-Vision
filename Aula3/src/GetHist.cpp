#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	int type;

	//presents options to user
	system("clear");		//clears terminal window
	cout << endl << "-----------------------" << endl;
	cout << " Histogram Calculation " << endl;
	cout << "-----------------------" << endl << endl;
	cout << " Choose image type (BGR = 0, Grayscale = 1, both = 2): ";
	cin >> type;

	//opens default camera
	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;

	//gets camera picture and displays it
	Mat frame;
	cap >> frame;
	namedWindow("Original Picture", 1);
	imshow("Original Picture", frame);

	//configures histograms
	int histSize = 256;							//sets number of columns
	float range[] = {0, 256};					//sets ranges for B, G and R
	const float* histRange = {range};			
	bool uniform = true, accumulate = false;	//initializes some 'calcHist' args

	//draws histograms
	int histW = 512, histH = 400;				//sets size of window
	int binW = cvRound((double)histW/histSize);	//sets size of columns
	Mat histImage(histH, histW, CV_8UC3, Scalar(0, 0, 0));

	if (type == 1 || type == 2) {
		//converts to grayscale and displays image
		Mat grayImage;
		cvtColor(frame, grayImage, CV_BGR2GRAY);
		namedWindow("Grayscale Picture", 1);
		imshow("Grayscale Picture", grayImage);

		Mat grayHist;

		//calculates histogram
		calcHist(&grayImage, 1, 0, Mat(), grayHist, 1, &histSize, &histRange, uniform, accumulate);
		
		//normalizes results to number of rows that the images has
		normalize(grayHist, grayHist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

		//draws grayscale channel (image, line points, color, thickness, 8-conn line, no shift)
		for (int i = 1; i < histSize; i++) {
			line(histImage, Point(binW*(i - 1), histH - cvRound(grayHist.at<float>(i - 1))),
							Point(binW*(i), histH - cvRound(grayHist.at<float>(i))),
							Scalar(255, 255, 255), 2, 8, 0);
		}
	}

	if (type == 0 || type == 2) {
		//splits image in three planes (BGR)
		vector<Mat> bgrPlanes;					//creates vector of arrays
		split(frame, bgrPlanes);				//stores split image's arrays in vector

		Mat bHist, gHist, rHist;

		//calculates histograms
		calcHist(&bgrPlanes[0], 1, 0, Mat(), bHist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgrPlanes[1], 1, 0, Mat(), gHist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgrPlanes[2], 1, 0, Mat(), rHist, 1, &histSize, &histRange, uniform, accumulate);

		//normalizes results to number of rows that the image has
		normalize(bHist, bHist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(gHist, gHist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(rHist, rHist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

		//draws for each channel
		for (int i = 1; i < histSize; i++) {
			line(histImage, Point(binW*(i - 1), histH - cvRound(bHist.at<float>(i - 1))),
							Point(binW*(i), histH - cvRound(bHist.at<float>(i))),
							Scalar(255, 0, 0), 2, 8, 0);
			line(histImage, Point(binW*(i - 1), histH - cvRound(gHist.at<float>(i - 1))),
							Point(binW*(i), histH - cvRound(gHist.at<float>(i))),
							Scalar(0, 255, 0), 2, 8, 0);
			line(histImage, Point(binW*(i - 1), histH - cvRound(rHist.at<float>(i - 1))),
							Point(binW*(i), histH - cvRound(rHist.at<float>(i))),
							Scalar(0, 0, 255), 2, 8, 0);
		}	
	}

	//displays histograms
	namedWindow("Histograms", 1);
	imshow("Histograms", histImage);

	waitKey(0);


	return 0;
}