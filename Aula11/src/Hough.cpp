#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/// Global variables

/** General variables */
Mat frame, frame_gray, edges;
Mat standard_hough;
int min_threshold = 50;
int max_trackbar = 150;
string filename = "vid/vid1.mp4";

const char* standard_name = "Standard Hough Lines Demo";

int s_trackbar = max_trackbar;
int p_trackbar = max_trackbar;

/// Function Headers
void Standard_Hough( int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
    system("clear");		//clears terminal window
	  cout << endl << "---------------" << endl;
	  cout << " Hough Transform - Camera Input" << endl;
    cout << endl << "---------------" << endl;

    //opens default camera
    VideoCapture cap(0);
    //check if success
    if (!cap.isOpened())
      {
        cout << "Error when reading camera!" << endl;
        return -1;
      }
    //namedWindow( "Video File", 1);
    for( ; ; )
    {
      cap >> frame;
      if(frame.empty())
        break;
      //imshow("Video File", frame);

      /// Pass the image to gray
      cvtColor( frame, frame_gray, COLOR_RGB2GRAY );

      /// Apply Canny edge detector
      Canny( frame_gray, edges, 50, 200, 3 );

      /// Create Trackbars for Thresholds
      char thresh_label[50];
      sprintf( thresh_label, "Thres: %d + input", min_threshold );

      namedWindow( standard_name, WINDOW_AUTOSIZE );
      createTrackbar( thresh_label, standard_name, &s_trackbar, max_trackbar, Standard_Hough);

      /// Initialize
      Standard_Hough(0, 0);

      waitKey(30);
      if( waitKey(24) >= 0 ) { break; }
      }

   return 0;
}

/**
 * @function Standard_Hough
 */
void Standard_Hough( int, void* )
{
  vector<Vec2f> s_lines;
  cvtColor( edges, standard_hough, COLOR_GRAY2BGR );

  /// 1. Use Standard Hough Transform
  HoughLines( edges, s_lines, 1, CV_PI/180, min_threshold + s_trackbar, 0, 0 );

  /// Show the result
  for( size_t i = 0; i < s_lines.size(); i++ )
     {
      float r = s_lines[i][0], t = s_lines[i][1];
      double cos_t = cos(t), sin_t = sin(t);
      double x0 = r*cos_t, y0 = r*sin_t;
      double alpha = 1000;

       Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
       Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
       line( standard_hough, pt1, pt2, Scalar(255,0,0), 3, LINE_AA);
     }

   imshow( standard_name, standard_hough );
}
