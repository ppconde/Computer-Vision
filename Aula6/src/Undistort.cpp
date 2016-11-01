#include <iostream>
#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


Mat undistImageL, undistImageR;
Mat F;

Mat lepi, repi;

vector<Point> ptsl, ptsr;


void drawEpiLines(Mat epiImage, Mat undistImage, string window) {
	// This function draws epipolar lines in an image
	for (int i = 0; i < epiImage.rows; i++) {
		//calculates line points
		float a = epiImage.at<float>(i, 0);
		float b = epiImage.at<float>(i, 1);
		float c = epiImage.at<float>(i, 2);

		double x = 0;
		double y = -(a/b)*x - (c/b);
		Point pt1 = Point(x, y);

		x = undistImageR.cols;
		y = -(a/b)*x - (c/b);
		Point pt2 = Point(x, y);

		//draws epipolar line
		line(undistImage, pt1, pt2, Scalar(0, 0, 255));

		imshow(window, undistImage);
	}
}

void mouseHandlerL(int event, int x, int y, int flags, void* param) {
	// Handles mouse clicking on the left image
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			Point selected = Point(x, y);
			ptsl.push_back(selected);

			Mat epiImage;
			computeCorrespondEpilines(ptsl, 1, F, epiImage);
			drawEpiLines(epiImage, undistImageR, "Undistorted Right");

			break;
	}
}

void mouseHandlerR(int event, int x, int y, int flags, void* param) {
	// Handles mouse clicking on the right image	
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			Point selected = Point(x, y);
			ptsr.push_back(selected);

			Mat epiImage;
			computeCorrespondEpilines(ptsr, 2, F, epiImage);
			drawEpiLines(epiImage, undistImageL, "Undistorted Left");
			
			break;
	}
}

int main(int argc, char **argv) {
	char filename[200];
	bool set;
	int pair;

	//chessboard properties
    int nBoards;                   //number of images

    //camera calibration params
    Mat camMatrix1 = Mat(3, 3, CV_32FC1);       //intrinsic matrixes
    Mat camMatrix2 = Mat(3, 3, CV_32FC1);
    Mat distCoeffs1, distCoeffs2;               //distortion coefficients matrix
    Mat R, T, E;

    Mat imageL, imageR;

	//presents user interface
    system("clear");
    cout << endl << "------------------" << endl;
    cout << " Stereo Undistort " << endl;
    cout << "------------------" << endl << endl;
    cout << " Choose set of pictures (1st set = 0, 2nd set = 1): ";
    cin >> set;
    cout << endl;

    //sets board properties
    if (set) {
    	nBoards = 31;

        sprintf(filename, "camParams2.xml");
    }
    else {
    	nBoards = 13;

        sprintf(filename, "camParams1.xml");
    }
    cout << " Choose pair of pictures [1 - " << nBoards << "]: ";

    //selects pair of images (left and right)
    cin >> pair;
    cout << endl;

    //imports camera calibration params from external XML file
    FileStorage fs(filename, FileStorage::READ);

  	if(!fs.isOpened()){
    	cerr << "Failed to open stereoParams.xml" << endl;
    	return 1;
  	}

  	fs["camMatrix1"] >> camMatrix1;
  	fs["camMatrix2"] >> camMatrix2;
  	fs["distCoeffs1"] >> distCoeffs1;
  	fs["distCoeffs2"] >> distCoeffs2;
	fs["fundamental"] >> F;

    //reads pair of images
    if (set) {
    	sprintf(filename, "img/StereoL%d.bmp", pair);
    	imageL = imread(filename, CV_LOAD_IMAGE_COLOR);
    	sprintf(filename, "img/StereoR%d.bmp", pair);
    	imageR = imread(filename, CV_LOAD_IMAGE_COLOR);	
    }
    else {
    	sprintf(filename, "img/left%02d.jpg", pair);
    	imageL = imread(filename, CV_LOAD_IMAGE_COLOR);
    	sprintf(filename, "img/right%02d.jpg", pair);
    	imageR = imread(filename, CV_LOAD_IMAGE_COLOR);    
    }

    if (!imageL.data || !imageR.data) {
        cout << " Could not load image files" << endl;
        return -1;
    }

    //applies undistort to images
    undistort(imageL, undistImageL, camMatrix1, distCoeffs1);
    undistort(imageR, undistImageR, camMatrix2, distCoeffs2);

    //selects original images or epipolar lines
	bool sel;
    cout << " Select to display original images (0)";
    cout << " or epipolar lines (1): ";
    cin >> sel;

    if (sel) {
    	imshow("Undistorted Left", undistImageL);
    	imshow("Undistorted Right", undistImageR);

    	setMouseCallback("Undistorted Left", mouseHandlerL);
    	setMouseCallback("Undistorted Right", mouseHandlerR);
    }
    else {
    	imshow("Original Left", imageL);
    	imshow("Original Right", imageR);
    	imshow("Undistorted Left", undistImageL);
    	imshow("Undistorted Right", undistImageR);
    }

    waitKey(0);

	return 0;
}
