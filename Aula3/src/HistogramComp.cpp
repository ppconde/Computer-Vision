#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;

int main(){

	//reads images
	Mat src1 = imread("./img/img1.jpg");
	Mat src2 = imread("./img/img2.jpg");

	//if no data read, aborts
	if (!src1.data){
		std::cout << "Error loading img1." << std::endl;
		return -1;
	}
	if (!src2.data){
		std::cout << "Error loading img2." << std::endl;
		return -1;
	}

	//creates windows
	namedWindow("Original Image 1", 1);
	namedWindow("Original Image 2", 1);

	//shows original images
	imshow("Original Image 1", src1);
	imshow("Original Image 2", src2);

  /// Separate the image in 3 places ( B, G and R )
  vector <Mat> bgr_planes;
  vector <Mat> bgr_planes2;
  split( src1, bgr_planes );
  split( src2, bgr_planes2 );

  /// Establish the number of bins
 int histSize = 256;

 /// Set the ranges ( for B,G,R) )
 float range[] = { 0, 256 } ;
 const float* histRange = { range };

 bool uniform = true; bool accumulate = false;

 Mat b_hist, g_hist, r_hist;
 Mat b_hist2, g_hist2, r_hist2;

 /// Compute the histograms:
 calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
 calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
 calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
 calcHist( &bgr_planes2[0], 1, 0, Mat(), b_hist2, 1, &histSize, &histRange, uniform, accumulate );
 calcHist( &bgr_planes2[1], 1, 0, Mat(), g_hist2, 1, &histSize, &histRange, uniform, accumulate );
 calcHist( &bgr_planes2[2], 1, 0, Mat(), r_hist2, 1, &histSize, &histRange, uniform, accumulate );

 // Draw the histograms for B, G and R
 int hist_w = 512; int hist_h = 400;
 int bin_w = cvRound( (double) hist_w/histSize );
 int hist_w2 = 512; int hist_h2 = 400;
 int bin_w2 = cvRound( (double) hist_w2/histSize );

 Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
 Mat histImage2( hist_h2, hist_w2, CV_8UC3, Scalar( 0,0,0) );

 /// Normalize the result to [ 0, histImage.rows ]
 normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
 normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
 normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
 normalize(b_hist2, b_hist2, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
 normalize(g_hist2, g_hist2, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
 normalize(r_hist2, r_hist2, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

 /// Draw for each channel
 for( int i = 1; i < histSize; i++ )
 {
                      //img1
     line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                      Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                      Scalar( 255, 0, 0), 2, 8, 0  );
     line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                      Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                      Scalar( 0, 255, 0), 2, 8, 0  );
     line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                      Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                      Scalar( 0, 0, 255), 2, 8, 0  );
                      //img2
      line( histImage2, Point( bin_w2*(i-1), hist_h2 - cvRound(b_hist2.at<float>(i-1)) ) ,
                       Point( bin_w2*(i), hist_h2 - cvRound(b_hist2.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage2, Point( bin_w2*(i-1), hist_h2 - cvRound(g_hist2.at<float>(i-1)) ) ,
                       Point( bin_w2*(i), hist_h2 - cvRound(g_hist2.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage2, Point( bin_w2*(i-1), hist_h2 - cvRound(r_hist2.at<float>(i-1)) ) ,
                       Point( bin_w2*(i), hist_h2 - cvRound(r_hist2.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
 }

  /// Display
  imshow("calcHist Img1", histImage );
  imshow("calcHist Img2", histImage2 );


  /// Apply the histogram comparison methods
  for( int i = 0; i < 4; i++ )
  {
      int compare_method = i;
      double compB = compareHist( b_hist, b_hist2, compare_method );
      double compG = compareHist( g_hist, g_hist2, compare_method );
      double compR = compareHist( r_hist, r_hist2, compare_method );

      printf( " Method [%d] Compare Blue, Compare Green, Compare Red: %f, %f, %f \n", i, compB, compG , compR);
  }

	//waits for input
	waitKey(0);
	return 0;
}
