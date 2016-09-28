#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
	//if the number of arguments given by terminal is not 1 (1+1=2), aborts
	if (argc != 2){
		cout << " Usage: LoadnDisplay <image path>" << endl;
		return -1;
	}

	//creates new "Mat" object to store image data
	Mat image;

	//initializes "image" by reading and loading image file data
	image = imread(argv[1], CV_LOAD_IMAGE_COLOR);	//loads color data

	//if no data given to image, aborts
	if (!image.data){
		cout << "Could not open or find the image." << endl;
		return -1;
	}

	//creates a window for image display
	namedWindow("Display Window", CV_WINDOW_AUTOSIZE);

	//presents image on window
	imshow("Display Window", image);

	//waits for keystroke in the window
	waitKey(0);

	return 0;
}