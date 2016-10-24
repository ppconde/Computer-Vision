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
    bool type, external;
    char filename[200];

    //chessboard properties
    int nBoards = 13;                   //number of images
    int wBoard = 9;
    int hBoard = 6;
    int boardSize = wBoard * hBoard;    //size of board

    //chessboard coordinates and image pixels
    vector<vector<Point3f> > objChessPoints;
    vector<vector<Point2f> > imgChessPoints;

    //corners detected in each image
    vector<Point2f> corners;

    //camera calibration params
    Mat intrinsicMatrix = Mat(3, 3, CV_32FC1);  //intrinsic matrix
    Mat distCoeffs;                             //distortion coefficients matrix
    vector<Mat> rvecs;                          //rotation vectors
    vector<Mat> tvecs;                          //translation vectors

    //coordinate system points
    vector<Point3f> objCoordPoints;
    objCoordPoints.push_back(Point3f(0, 0, 0));
    objCoordPoints.push_back(Point3f(0, 0, 1));
    objCoordPoints.push_back(Point3f(1, 0, 0));
    objCoordPoints.push_back(Point3f(0, 1, 0));
    vector<Point2f> imgCoordPoints;

    int cornerCount;

    Mat image, objDisplay;

    //presents user interface
    system("clear");
    cout << endl << "--------------------" << endl;
    cout << " Camera Calibration " << endl;
    cout << " Choose type of calibration [Internal = 0, External = 1]: ";
    cin >> external;
    cout << endl;

    //creates vector with preliminary coordinates of points
    vector<Point3f> obj;

    for(int i = 0; i < boardSize; i++)
    {
        obj.push_back(Point3f(float(i/wBoard), float(i%wBoard), 0.0));
    }

    if(!external)
    {
          system("clear");                    //clears terminal window
          cout << endl << "--------------------" << endl;
          cout << " Camera Calibration " << endl;
          cout << "--------------------" << endl << endl;
          cout << " Choose calibration method [still images = 0, video feed = 1]: ";
          cin >> type;
          cout << endl;
          if (type)
          {
              int n = 0;

              //opens default camera
              VideoCapture cap(0);

              //check if success
              if (!cap.isOpened())
              {
                  cout << " Could not read video feed." << endl;
                  return -1;
              }

              //finds and displays chessboard points from video
              for(;;)
              {
                  cap >> image;

                  //finds and displays corners
                  cornerCount = FindAndDisplayChessboard(image, wBoard, hBoard, &corners);

                  //transfers data to objects display vectors
                  if (cornerCount == boardSize)
                  {
                      imgChessPoints.push_back(corners);
                      objChessPoints.push_back(obj);

                      //extracts camera calibration params
                      calibrateCamera(objChessPoints, imgChessPoints, image.size(), intrinsicMatrix, distCoeffs, rvecs, tvecs, 0);

                      cap >> objDisplay;

                      projectPoints(objCoordPoints, rvecs.at(n), tvecs.at(n), intrinsicMatrix, distCoeffs, imgCoordPoints);
                      line(objDisplay, Point(imgCoordPoints.at(0)), Point(imgCoordPoints.at(1)), Scalar(0, 0, 255), 2, 8);
                      line(objDisplay, Point(imgCoordPoints.at(0)), Point(imgCoordPoints.at(2)), Scalar(0, 0, 255), 2, 8);
                      line(objDisplay, Point(imgCoordPoints.at(0)), Point(imgCoordPoints.at(3)), Scalar(0, 0, 255), 2, 8);

                      imshow("Output", objDisplay);

                      n++;
                  }
                  else
                  {
                      cap >> objDisplay;
                      imshow("Output", objDisplay);
                  }

                  if (waitKey(5) >= 0) break;
              }
          }
          else
          {
              //finds and displays chessboard points
              for (int i = 0; i < nBoards; i++)
              {
                  //reads image
                  sprintf(filename, "img/%02d.jpg", i + 1);
                  cout << " Reading " << filename << endl;
                  image = imread(filename, CV_LOAD_IMAGE_COLOR);

                  if(!image.data)
                  {
                      cout << " Could not load image file: " << filename << endl;
                      return -1;
                  }

                  //finds and displays corners
                  cornerCount = FindAndDisplayChessboard(image, wBoard, hBoard, &corners);

                  //transfers data to objects display vectors
                  if (cornerCount == boardSize)
                  {
                      imgChessPoints.push_back(corners);
                      objChessPoints.push_back(obj);

                      //extracts camera calibration params
                      calibrateCamera(objChessPoints, imgChessPoints, image.size(), intrinsicMatrix, distCoeffs, rvecs, tvecs, 0);

                      objDisplay = imread(filename, CV_LOAD_IMAGE_COLOR);

                      projectPoints(objCoordPoints, rvecs.at(i), tvecs.at(i), intrinsicMatrix, distCoeffs, imgCoordPoints);
                      line(objDisplay, Point(imgCoordPoints.at(0)), Point(imgCoordPoints.at(1)), Scalar(0, 0, 255), 2, 8);
                      line(objDisplay, Point(imgCoordPoints.at(0)), Point(imgCoordPoints.at(2)), Scalar(0, 0, 255), 2, 8);
                      line(objDisplay, Point(imgCoordPoints.at(0)), Point(imgCoordPoints.at(3)), Scalar(0, 0, 255), 2, 8);

                      imshow("Output", objDisplay);
                  }
                  else
                  {
                      objDisplay = imread(filename, CV_LOAD_IMAGE_COLOR);
                      imshow("Output", objDisplay);
                  }

                  waitKey(0);
              }
          }

    }
    else
    {
            // load Matrixes
      Mat intrinsic_matrix;
      Mat distortion_coeffs;
      FileStorage fs("../camParams.xml", FileStorage::READ);
      if (!fs.isOpened())
      {
      cerr << "Failed to open " << filename << endl;
      return 1;
      }
      fs["cameraMatrix"] >> intrinsic_matrix;
      fs["distCoeffs"] >> distortion_coeffs;
      fs.release();
    }

    //exports camera calibration params to extrenal XML file
    FileStorage fs("camParams.xml", FileStorage::WRITE);
    fs << "cameraMatrix" << intrinsicMatrix << "distCoeffs" << distCoeffs;
    fs.release();

    return 0;
}
