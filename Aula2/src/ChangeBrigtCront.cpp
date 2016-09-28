#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

double alpha;
int beta;

int main(int argc, char** argv){
	//reads image
	Mat image = imread(argv[1]);
	Mat new_image = Mat::zeros(image.size(), image.type());

	//initializes values
	std::cout << "Basic Linear Transformations " << std::endl;
	std::cout << "-----------------------------" << std::endl;
	std::cout << " Enter alpha value [0.0-3.0]: "; std::cin >> alpha;
	std::cout << " Enter beta value [0-100]: "; std::cin >> beta;

	//new_image(i,j) = alpha*image(i,j) + beta
	for (int y = 0; y < image.rows; y++){
		for (int x = 0; x <image.cols; x++){
			for (int c = 0; c < 3; c++){
				new_image.at<Vec3b>(y, x)[c] = 
				  saturate_cast<uchar>(alpha * (image.at<Vec3b>(y, x)[c]) + beta);  
			}
		}
	}

	//creates windows
	namedWindow("Original Image", 1);
	namedWindow("New Image", 1);

	//shows images
	imshow("Original Image", image);
	imshow("New Image", new_image);

	//waits for input
	waitKey(0);

	return 0;
}