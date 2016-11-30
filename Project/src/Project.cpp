#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void mouseHandler(int event, int x, int y, int flags, void* param);


vector<Point2f> prevPts, nextPts;		//frame features
vector<uchar> status;					//status vector
vector<float> err;						//error vector

int nPts = 500;						//number of features
double qLevel = 0.5;				//quality level
double minDist = 0.05;				//min euclidian distance

int drag = 0, select_flag = 0;
Mat frame, prevFrame, nextFrame;		//image matrixes
Point point1, point2;
bool callback = false;

int main() {

  //video file
	VideoCapture cap;
	const char* video = "vid/vid1.mp4";
  cap.open(video);

	//check if success
	if (!cap.isOpened()) return -1;

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl << " --------------------------------" << endl;
	cout << "  Optical Flow with Lucas-Kanade " << endl;
	cout << " --------------------------------" << endl << endl;

	namedWindow("Optical Flow L-K", CV_WINDOW_AUTOSIZE);

	//gets initial image
	cap >> frame;
	cvtColor(frame, nextFrame, CV_BGR2GRAY);

	for (;;) {
		//gets image features
		goodFeaturesToTrack(nextFrame, nextPts, nPts, qLevel, minDist);

		prevFrame = nextFrame.clone();		//first frame is the same as last one
		prevPts = nextPts;
		cap >> frame;						//gets a new frame from camera

		cvtColor(frame, nextFrame, CV_BGR2GRAY);

		//calculates optical flow using Lucas-Kanade
		calcOpticalFlowPyrLK(prevFrame, nextFrame, prevPts, nextPts, status, err);

		//draws motion lines on display
		for (int i = 0; i < nextPts.size(); i++) {
			if (status[i]) {
				line(frame, prevPts[i], nextPts[i], Scalar(255, 0, 0));
			}
		}

		imshow("Optical Flow L-K", frame);	//shows original capture
		setMouseCallback("frame",mouseHandler,0);
		if(!frame.data) break;
	    if (waitKey(50) >= 0) break;				//waits 5ms for program to render next frame
	}

	return 0;
}
void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
        point1 = Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        Mat img1 = frame.clone();
        point2 = Point(x, y);
        rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow(frame, img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag && !select_flag)
    {
        Mat img2 = frame.clone();
        point2 = Point(x, y);
        drag = 0;
        select_flag = 1;
        imshow(frame, img2);
        callback = true;
    }
}
