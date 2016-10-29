#include <iostream>
#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


Mat finalL, finalR;

Mat F;


void mouseHandler(int event, int x, int y, int flags, void* param) {
	// Handles mouse clicking on an image
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			//draws lines
			Point pt1 = Point(0, y);
            Point pt2 = Point(finalL.cols, y);
            line(finalL, pt1, pt2, Scalar(0, 0, 255));
            line(finalR, pt1, pt2, Scalar(0, 0, 255));

            imshow("Rectified Left", finalL);
            imshow("Rectified Right", finalR);
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

    //image rectification matrixes
    Mat map1x, map1y, map2x, map2y;
    Mat R1, R2, P1, P2;
    Mat Q;

    Mat imageL, imageR, undistImageL, undistImageR;;

	//presents user interface
    system("clear");
    cout << endl << "------------------" << endl;
    cout << " Stereo Undistort " << endl;
    cout << "------------------" << endl << endl;
    cout << " Choose set of pictures (1st set = 0, 2nd set = 1) ";
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
    cout << " Choose pair of pictures [1 - " << nBoards << "] ";

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
    fs["rotation"] >> R;
    fs["translation"] >> T;
    fs["essential"] >> E;
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

    //rectifies images
    stereoRectify(camMatrix1, distCoeffs1, camMatrix2, distCoeffs2,
        undistImageL.size(), R, T, R1, R2, P1, P2, Q, 0);

    //computes transformation between original and rectified image
    initUndistortRectifyMap(camMatrix1, distCoeffs1, R1, P1,
        undistImageL.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(camMatrix2, distCoeffs2, R2, P2,
        undistImageR.size(), CV_32FC1, map2x, map2y);

    //applies transformation to image
    remap(undistImageL, finalL, map1x, map1y, INTER_LINEAR);
    remap(undistImageR, finalR, map2x, map2y, INTER_LINEAR);

    //selects original imagee comparison or line display
	bool sel;
    cout << " Select to display original images (0)";
    cout << " or comparison lines (1): ";
    cin >> sel;

    if (sel) {
    	imshow("Rectified Left", finalL);
    	imshow("Rectified Right", finalR);

    	setMouseCallback("Rectified Left", mouseHandler);
    	setMouseCallback("Rectified Right", mouseHandler);
    }
    else {
    	imshow("Original Left", imageL);
    	imshow("Original Right", imageR);
    	imshow("Rectified Left", finalL);
    	imshow("Rectified Right", finalR);
    }

    waitKey(0);

	return 0;
}