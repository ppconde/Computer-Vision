#include <iostream>
#include <vector>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int FindAndDisplayChessboard(Mat image, int wBoard, int hBoard, vector<Point2f> *corners) {
    // This function finds corners in a chessboard with wBoard * hBoard dims,
    //displays them in image and returns number of detected corners.

    Size boardSize = Size(wBoard, hBoard);

    Mat grayImage;
    cvtColor(image, grayImage, CV_BGR2GRAY);

    //finds chessboard corners
    bool found = findChessboardCorners(grayImage, boardSize, *corners, 0);
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    cornerSubPix(grayImage, *corners, Size(5, 5), Size(-1, -1), criteria);

    //draws results
    if (found) {
        drawChessboardCorners(image, boardSize, Mat(*corners), found);
    }

    imshow("Calibration", image);
    cout << " Number of corners: " << corners -> size() << endl;

    return corners -> size();
}

int main(int argc, char **argv) {
    char filename[200];
    bool set;

    //chessboard properties
    int nBoards;                   //number of images
    int wBoard;
    int hBoard;

    //matrixes for stereo calibration
    vector<vector<Point2f> > ipts1;
    vector<vector<Point2f> > ipts2;
    vector<vector<Point3f> > opts;

    //camera calibration params
    Mat camMatrix1 = Mat(3, 3, CV_32FC1);       //intrinsic matrixes
    Mat camMatrix2 = Mat(3, 3, CV_32FC1);
    Mat distCoeffs1, distCoeffs2;               //distortion coefficients matrix
    Mat R, T, E, F;

    //points to consider for each image
    int npoints;

    //corners detected in each image
    vector<Point2f> corners;

    Mat image, objDisplay;

    //presents user interface
    system("clear");
    cout << endl << "--------------------" << endl;
    cout << " Stereo Calibration " << endl;
    cout << "--------------------" << endl << endl;
    cout << " Choose set of pictures (1st set = 0, 2nd set = 1) ";
    cin >> set;
    cout << endl;

    //chessboard properties' initialization
    if (set) {
        nBoards = 31;
        wBoard = 10;
        hBoard = 7;
    }
    else {
        nBoards = 13;
        wBoard = 9;
        hBoard = 6;
    }
    int boardSize = wBoard * hBoard;    //size of board

    //creates vector with preliminary coordinates of points
    vector<Point3f> obj;
    for(int i = 0; i < boardSize; i++) {
        obj.push_back(Point3f(float(i/wBoard), float(i%wBoard), 0.0));
    }

    //finds and displays chessboard points
    for (int i = 0; i < 2*nBoards; i++) {
        //reads image
        if (i < nBoards) {
            if (set) sprintf(filename, "img/StereoL%d.bmp", i + 1);
            else sprintf(filename, "img/left%02d.jpg", i + 1);    
        }
        else {
            if (set) sprintf(filename, "img/StereoR%d.bmp", i - nBoards + 1);
            else sprintf(filename, "img/right%02d.jpg", i - nBoards + 1);
        }
        cout << " Reading \"" << filename << "\"" << endl;
        image = imread(filename, CV_LOAD_IMAGE_COLOR);

        if(!image.data) {
            cout << " Could not load image file: " << filename << endl;
            return -1;
        }

        if (i < nBoards ) {
            //finds and displays corners
            npoints = FindAndDisplayChessboard(image, wBoard, hBoard, &corners);

            //transfers data to objects display vectors
            if (npoints == boardSize) {
                ipts1.push_back(corners);
                opts.push_back(obj);
            }
        }
        else {
            //finds and displays corners
            npoints = FindAndDisplayChessboard(image, wBoard, hBoard, &corners);

            //transfers data to objects display vectors
            if (npoints == boardSize) {
                ipts2.push_back(corners);
            }
        }
    }

    //calibrates stereo cameras
    TermCriteria criteria2 = TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5);
    double rms = stereoCalibrate(opts, ipts1, ipts2, camMatrix1, distCoeffs1, camMatrix2, distCoeffs2, image.size(), R, T, E, F, CV_CALIB_SAME_FOCAL_LENGTH, criteria2);

    //exports camera calibration params to external XML file
    if (set) sprintf(filename, "camParams2.xml");
    else sprintf(filename, "camParams1.xml");
    FileStorage fs(filename, FileStorage::WRITE);
    fs << "camMatrix1" << camMatrix1;
    fs << "camMatrix2" << camMatrix2;
    fs << "distCoeffs1" << distCoeffs1;
    fs << "distCoeffs2" << distCoeffs2;
    fs << "rotation" << R;
    fs << "translation" << T;
    fs << "essential" << E;
    fs << "fundamental" << F;
    fs.release();

    waitKey(0);

    return 0;
}
