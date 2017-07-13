#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

//namespace declaration
using namespace std;
using namespace cv;

#define bitdepth CV_32F

Mat frame, snakeFrame;
char file[20];
const float wl_max = 1.0, we_max = 1.0, wt_max = 1.0;
float wl = 0.5, we = 0.3, wt = 0.5;
//char TrackBarwl[50], TrackBarwe[50], TrackBarwt[50];

Mat config_eext(float wl, float we, float wt, Mat image);
//static void on_TrackBar( int, void* );

int main(int argc, char** argv){

  //video file selection
  if (argc == 2) sprintf(file, "%s", argv[1]);

  VideoCapture cap;

	cap.open(file);

  //check if success
	if (!cap.isOpened()) {
		cout << "Error: video file could not be loaded. Aborting." << endl << endl;
		return -1;
	}

   for(;;){
     cap >> frame;		//gets new frame
     if(!frame.data) {
       cout << "Error: no video data found." << endl << endl;
       break;
     }
     snakeFrame = config_eext(wl, we, wt, frame);
     imshow("frame", frame);
     imshow("snakeFrame", snakeFrame);

     char key = waitKey(33);

     //if "space" pressed, pauses
 	    if (key == 32) {
 	    	key = 0;

        while (key != 32) {
 	    		key = waitKey(5);

          if(key == 27)  break;
        }
      }
    }
   return 0;
}

Mat config_eext(float wl, float we, float wt, Mat image)
{
  Mat eline, gradx, grady, img_gray, eedge;

  //bitdepth defined as CV_32F
  image.convertTo(img_gray, bitdepth);

  //Convolution Kernels
  Mat m1, m2, m3, m4, m5;
  m1 = (Mat_<float>(1, 2) << 1, -1);
  m2 = (Mat_<float>(2, 1) << 1, -1);
  m3 = (Mat_<float>(1, 3) << 1, -2, 1);
  m4 = (Mat_<float>(3, 1) << 1, -2, 1);
  m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);

  img_gray.copyTo(eline);

  //Kernels de gradiente
  Mat kernelx = (Mat_<float>(1, 3) << -1, 0, 1);
  Mat kernely = (Mat_<float>(3, 1) << -1, 0, 1);

  //Gradiente em x e em y
  filter2D(img_gray, gradx, -1, kernelx);
  filter2D(img_gray, grady, -1, kernely);

  //Edge Energy como definido por Kass
  eedge = -1 * (gradx.mul(gradx) + grady.mul(grady));

  //Termination Energy Convolution
  Mat cx, cy, cxx, cyy, cxy, eterm(img_gray.rows, img_gray.cols, bitdepth), cxm1, den, cxcx, cxcxm1, cxcxcy, cxcycxy, cycycxx;
  filter2D(img_gray, cx, bitdepth, m1);
  filter2D(img_gray, cy, bitdepth, m2);
  filter2D(img_gray, cxx, bitdepth, m3);
  filter2D(img_gray, cyy, bitdepth, m4);
  filter2D(img_gray, cxy, bitdepth, m5);

  //element wise operations to find Eterm
  cxcx = cx.mul(cx);
  cxcx.convertTo(cxcxm1, -1, 1, 1);
  den = cxcxm1 + cy.mul(cy);
  cv::pow(den, 1.5, den);
  cxcxcy = cxcx.mul(cy);
  cxcycxy = cx.mul(cy);
  cxcycxy = cxcycxy.mul(cxy);
  cycycxx = cy.mul(cy);
  cycycxx = cycycxx.mul(cxx);
  eterm = (cxcxcy - 2 * cxcycxy + cycycxx);
  cv::divide(eterm, den, eterm, -1);

  //Image energy
  Mat eext;
  eext = wl*eline + we*eedge + wt*eterm;
  return eext;
}
