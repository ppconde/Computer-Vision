#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;
using namespace cv;

char file[20];

/**
 * @function main
 */
int main( int argc, char** argv )
{
  //video file
	VideoCapture cap;

	sprintf(file, "vid/vid1.mp4");
	//const char* video = "vid/vid1.mp4";
	cap.open(file);

  /// Establish the number of bins
  int histSize = 4;

  Mat frame, frame1, frame2;
  long frameOne = 60;
  long frameTwo = 200;
  vector <Mat> N;
  N.push_back(1, 2, 3, 4, 5, 6);
  vector <Mat> S;
  N.push_back(5, 3, 4, 2, 1, 6);
  vector <Mat> E;
  N.push_back(6, 4, 1, 5, 1, 2);
  vector <Mat> W;
  N.push_back(3, 6, 5, 4, 2, 1);

  cap.set(CV_CAP_PROP_POS_FRAMES,frameOne);
  cap >> frame;
  frame1 = frame.clone();

  cap.set(CV_CAP_PROP_POS_FRAMES,frameTwo);
  cap >> frame;
  frame2 = frame.clone();

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 10 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  Mat n_hist, s_hist, e_hist, w_hist;

  /// Compute the histograms:
  calcHist( &N, 1, 0, Mat(), n_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &S, 1, 0, Mat(), s_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &E, 1, 0, Mat(), e_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &W, 1, 0, Mat(), w_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(n_hist, n_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(s_hist, s_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(e_hist, e_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(w_hist, w_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel

  /// Display
  //namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
  //imshow("calcHist Demo", histImage );

  waitKey(0);

  return 0;
}
