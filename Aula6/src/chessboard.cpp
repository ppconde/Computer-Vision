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

    //chessboard properties
    int nBoards = 10;                   //number of images
    int wBoard = 9;
    int hBoard = 6;
    int boardSize = wBoard * hBoard;    //size of board

    //chessboard coordinates and image pixels
    vector<vector<Point3f> > objChessPoints;
    vector<vector<Point2f> > imgChessPoints;

    //corners detected in each image
    vector<Point2f> corners;
    int cornerCount;

    Mat image, objDisplay;

    //presents user interface
    system("clear");
    cout << endl << "------------" << endl;
    cout << " Chessboard " << endl;
    cout << "------------" << endl << endl;

    //creates vector with preliminary coordinates of points
    vector<Point3f> obj;
    for(int i = 0; i < boardSize; i++) {
        obj.push_back(Point3f(float(i/wBoard), float(i%wBoard), 0.0));
    }

    //finds and displays chessboard points
    for (int i = 0; i < nBoards; i++) {
        //reads image
        sprintf(filename, "img/right%02d.jpg", i + 1);
        cout << " Reading \"" << filename << "\"" << endl;
        image = imread(filename, CV_LOAD_IMAGE_COLOR);

        if(!image.data) {
            cout << " Could not load image file: " << filename << endl;
            return -1;
        }

        //finds and displays corners
        cornerCount = FindAndDisplayChessboard(image, wBoard, hBoard, &corners);

        //transfers data to objects display vectors
        if (cornerCount == boardSize) {
            imgChessPoints.push_back(corners);
            objChessPoints.push_back(obj);
        }

        waitKey(0);
    }

    return 0;
}
