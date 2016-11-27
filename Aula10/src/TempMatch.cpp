#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define N_TEMPL		5		//num of templates
#define N_METHODS	6		//num of template matching methods
#define N_SCALE		5		//num of image sizes
#define MIN_SIZE	0.5		//minimum image scale
#define MAX_SIZE	2		//maximum image scale


int getMatch(Mat inImg, Mat templ, Mat& outImg, double matchVals[], Point& matchLoc, int method) {
	// This function performs template matching, using the method
	//passed as argument
	if (method >= 0 && method < N_METHODS) {
		double minVal, maxVal;
		Point minLoc, maxLoc;
		
		//matches template with image
		matchTemplate(inImg, templ, outImg, method);

		//finds minimum and maximum values locations
		minMaxLoc(outImg, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

		outImg = inImg;

		//stores location if better match is found
		if ((method == 0) || (method == 1)) {
			if (minVal < matchVals[0]) {
				matchVals[0] = minVal;
				matchLoc = minLoc;
				
				return 1;
			}
		}
		else {
			if (maxVal > matchVals[1]) {
				matchVals[1] = maxVal;
				matchLoc = maxLoc;
				
				return 1;
			}
		}

		return 0;
	}
	else return -1;
}

void drawMatch(Mat& inImg, Point pt, int tH, int tW, float ratio) {
	// This function draws a rectangle identifying a template match
	Point pt1, pt2;

	pt1 = Point(int(pt.x/ratio), int(pt.y/ratio));
	pt2 = Point(int((pt.x + tW)/ratio), int((pt.y + tH)/ratio));
	
	rectangle(inImg, pt1, pt2, Scalar(255, 0, 0), 2);
}

int main() {
	char str[20];
	bool sel;
	int fout, method, toRead, tH[N_TEMPL], tW[N_TEMPL];
	float ratio;	//ratio of old to new image width after scaling
	double matchVals[2] = {1e+10, 0}; 	//template matching values

	Point matchLoc;

	Mat inImg, outImg, resImg, templ[N_TEMPL];

	float incr = (MAX_SIZE - MIN_SIZE)/N_SCALE; //scale increment

	//presents user interface
	system("clear");		//clears terminal window
	cout
	<< endl
	<< " -------------------" << endl
	<< "  Template Matching " << endl
	<< " -------------------" << endl
	<< endl
	<< " Choose to match template in image [0] or video file [1]: ";
	cin >> sel;
	if (!sel) {
		cout
		<< endl
		<< " Choose image file [1...6]: ";
		cin >> toRead;
	}
	cout
	<< endl
	<< " Choose matching method: " << endl
	<< "  (0 = Square differences)" << endl
	<< "  (1 = Normalized square differences)" << endl
	<< "  (2 = Cross correlation)" << endl
	<< "  (3 = Normalized cross correlation)" << endl
	<< "  (4 = Cross coefficients)" << endl
	<< "  (5 = Normalized cross coefficients): ";
	cin >> method;
	cout << endl;

	//reads and displays template image(s)
	for (int i = 0; i < N_TEMPL; i++) {
		sprintf(str, "img/signs/sign%d.jpg", i + 1);
		templ[i] = imread(str);
		tH[i] = templ[i].rows;
		tW[i] = templ[i].cols;

		imshow("Template", templ[i]);
		waitKey(0);
	}
	destroyWindow("Template");

	if (sel) {
		const char* video = "vid/Bike.mp4";

		//initializes video capture
		VideoCapture cap(video);
		if (!cap.isOpened()) {
			cout << "Error: could not load video file." << endl;
			return -1;
		}

		for (;;) {
			cap >> inImg;		//gets new frame
			if (inImg.rows == 0 || inImg.cols == 0) break;
			for (int j = 0; j < N_TEMPL; j++) {
				int n = 0;
				ratio = 0;
				matchVals[0] = 1e+10;
				matchVals[1] = 0;
				for (float i = MIN_SIZE; i <= MAX_SIZE; i += incr) {
					//resizes input image
					resize(inImg, resImg, Size(), i, i);
					if ((resImg.rows < tH[j]) || (resImg.cols < tW[j])) {
						cout << "Error: scaled image is smaller than template(s)" << endl;
						return -1;
					}
					
					//performs template matching
					fout = getMatch(resImg, templ[j], outImg, matchVals, matchLoc, method);
					
					//associates best results with their scale ratio
					if (fout == 1) ratio = i;
					else if (fout == -1) {
						cout << "Error: invalid method used." << endl;
						return -1;
					}
					else {
						//consecutive bad detections = match already found
						n++;
						if (n == 2) break;
					}
				}

				//creates identifying rectangle
				drawMatch(inImg, matchLoc, tH[j], tW[j], ratio);
			}
			imshow("Template Matching", inImg);

			if (waitKey(5) >= 0) break;
		}
	}
	else {
		sprintf(str, "img/traffic%d.jpg", toRead);

		inImg = imread(str);
		if (!inImg.data) {
			cout << "Error: could not read image." << endl;
			return -1;
		}

		for (int j = 0; j < N_TEMPL; j++) {
			int n = 0;
			ratio = 0;
			matchVals[0] = 1e+10;
			matchVals[1] = 0;
			for (float i = MIN_SIZE; i <= MAX_SIZE; i += incr) {
				//resizes input image
				resize(inImg, resImg, Size(), i, i);
				if ((resImg.rows < tH[j]) || (resImg.cols < tW[j])) {
					cout << "Error: scaled image is smaller than template(s)" << endl;
					return -1;
				}
				
				//performs template matching
				fout = getMatch(resImg, templ[j], outImg, matchVals, matchLoc, method);

				//associates best results with their scale ratio
				if (fout == 1) ratio = i;
				else if (fout == -1) {
					cout << "Error: invalid method used." << endl;
					return -1;
				}
				else {
					//consecutive bad detections = match already found
					n++;
					if (n == 2) break;
				}
			}

			//creates identifying rectangle
			drawMatch(inImg, matchLoc, tH[j], tW[j], ratio);
		}
		imshow("Template Matching", inImg);

		waitKey(0);
	}
	
	return 0;
}