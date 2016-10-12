#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	char* windowName;
	int type;
	int ksize;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	//opens default camera
	VideoCapture cap(0);
	
	//check if success
	if (!cap.isOpened()) return -1;

	//presents options to user
	system("clear");		//clears terminal window
	cout << endl << "---------------------" << endl;
	cout << " Gradient Calculator " << endl;
	cout << "---------------------" << endl << endl;
	cout << " Choose image gradient calculation type (sobel = 0, sharr = 1, laplacian = 2: ";
	cin >> type;

	if (type != 1) {
		cout << " Choose Sobel kernel size [1, 3, 5 or 7]: ";
		cin >> ksize;
	}

	//creates horz and vert convolution matrixes
	Mat gradX, gradY;
	Mat absGradX, absGradY;

	Mat grayscale, final;

	namedWindow("Video Capture", 1);

	for(;;) {
		Mat frame;
		cap >> frame;								//gets a new frame from camera
		imshow("Video Capture", frame);				//shows original capture

		//applies gaussian blur
		GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT);

		//converts to grayscale
	    cvtColor(frame, grayscale, CV_BGR2GRAY);	//converts to grayscale
	    
		if (type == 1) {
			//calculates gradients using Scharr
			Scharr(grayscale, gradX, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
			Scharr(grayscale, gradY, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);

			windowName = "Scharr";
		}
		else if (type == 2) {
			Laplacian(grayscale, final, ddepth, ksize, scale, delta, BORDER_DEFAULT);
			convertScaleAbs(final, final);

			windowName = "Laplacian";
		}	    
		else {
			//calculates gradients using Sobel
			Sobel(grayscale, gradX, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
			Sobel(grayscale, gradY, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);

			windowName = "Sobel";
		}

		if (type == 0 || type == 1) {
			convertScaleAbs(gradX, absGradX);
			convertScaleAbs(gradY, absGradY);
			addWeighted(absGradX, 0.5, absGradY, 0.5, 0, final);
		}

	    imshow(windowName, final);
	    
	    if (waitKey(5) >= 0) break;				//waits 30ms for program to render next frame
	}

	return 0;
}