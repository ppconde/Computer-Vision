#include <iostream>
#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

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
    Mat R, T, E, F;

    //image rectification matrixes
    Mat map1x, map1y, map2x, map2y;
    Mat R1, R2, P1, P2;
    Mat Q;

    Mat imageL, imageR, undistImageL, undistImageR, finalL, finalR;

	//presents user interface
    system("clear");
    cout << endl << "----------------------" << endl;
    cout << " Image Reconstruction " << endl;
    cout << "----------------------" << endl << endl;
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

    //conversion to grayscale
    cvtColor(undistImageL, undistImageL, CV_BGR2GRAY);
    cvtColor(undistImageR, undistImageR, CV_BGR2GRAY);

    //applies transformation to image
    remap(undistImageL, finalL, map1x, map1y, INTER_LINEAR);
    remap(undistImageR, finalR, map2x, map2y, INTER_LINEAR);

    //stereoBM configuration
    Mat imgDisparity8U;
    Mat imgDisparity16S;
    int nDisparities = 16*5;
    int SADWindowSize = 21;

    //computes stereo correspondence between rectified images
    Ptr<StereoBM> sbm;
    sbm = StereoBM::create(nDisparities, SADWindowSize);
    sbm -> compute(finalL, finalR, imgDisparity16S);

    double minVal, maxVal;

    minMaxLoc(imgDisparity16S, &minVal, &maxVal);

    cout << "Min value: " << minVal << endl;
    cout << "Max value: " << maxVal << endl << endl;

    //outputs stereoBM result
    imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
    imshow("Disparity", imgDisparity8U);

    //gets 3D coordinates
    Mat points3D;
    reprojectImageTo3D(imgDisparity8U, points3D, Q, 0, -1);

    //saves 3D coordinates in XML file
    FileStorage file("3Dpoints.xml", FileStorage::WRITE);
    file << "coordinates" << points3D;
    file.release();

    waitKey(0);

	return 0;
}