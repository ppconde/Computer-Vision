#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
	//reads terminal argument string
	char* imageName = argv[1];

	//creation and init of "Mat" object with term arg data
	Mat image;
	image = imread(imageName, 1);

	//if terminal arg insuficient or no image data, aborts
	if (argc != 2 || !image.data){
		cout << "Could not find image data." << endl;
		return -1;
	}

	//creation of object for gray image
	Mat grayImage;

	//converts color Mat object into grayscale Mat object
	cvtColor(image, grayImage, CV_BGR2GRAY);

	//writes grayscale image
	imwrite("img/grayImage.jpg", grayImage);

	//displays both color and grayscale images
	namedWindow(imageName, CV_WINDOW_AUTOSIZE);
	namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);

	imshow(imageName, image);
	imshow("Gray Image", grayImage);

	waitKey(0);

	return 0;
}