#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

int main(){
	double alpha = 0.5, beta, input;
	Mat src1, src2, dst;

	//outputs text and accepts input as alpha value
	std::cout << " Simple Line Blender " << std::endl;
	std::cout << "---------------------" << std::endl;
	std::cout << "* Enter alpha [0-1]: ";
	std::cin >> input;

	if (input >= 0.0 && input <= 1.0) alpha = input;

	//reads images
	src1 = imread("../img/img1.jpg");
	src2 = imread("../img/img2.jpg");

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
	namedWindow("Linear Blend", 1);

	//shows original images
	imshow("Original Image 1", src1);
	imshow("Original Image 2", src2);

	//creates new image as weighted blend of other two
	beta = (1.0 - alpha);
	addWeighted(src1, alpha, src2, beta, 0.0, dst);

	//show new image in window
	imshow("Linear Blend", dst);

	//waits for input
	waitKey(0);
	return 0;
}