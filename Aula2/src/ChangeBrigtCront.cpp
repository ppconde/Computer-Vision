#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

double alpha;
int beta;

int main(int argc, char** argv){
	//reads image from file
	Mat image1 = imread(argv[1]);
	Mat new_image = Mat::zeros(image1.size(), image1.type());

	//initializes values
	std::cout << "Basic Linear Transformations " << std::endl;
	std::cout << "-----------------------------" << std::endl;
	std::cout << " Enter alpha value [0.0-3.0]: "; std::cin >> alpha;
	std::cout << " Enter beta value [0-100]: "; std::cin >> beta;

	//new_image(i,j) = alpha*image(i,j) + beta
	for (int y = 0; y < image1.rows; y++){
		for (int x = 0; x <image1.cols; x++){
			for (int c = 0; c < 3; c++){
				new_image.at<Vec3b>(y, x)[c] = 
				  saturate_cast<uchar>(alpha * (image1.at<Vec3b>(y, x)[c]) + beta);  
			}
		}
	}

	//creates windows
	namedWindow("Original Image", 1);
	namedWindow("New Image", 1);

	//shows images
	imshow("Original Image", image1);
	imshow("New Image", new_image);

	//reads image from computer camera
	VideoCapture cap(0);				//initializes camera
	if (!cap.isOpened()) return -1;		//if not initialized, aborts

	Mat image2;
	namedWindow("Camera Picture", 1);

	cap >> image2;						//captures camera frame

	imshow("Camera Picture", image2);

	//waits for input
	waitKey(0);

	return 0;
}