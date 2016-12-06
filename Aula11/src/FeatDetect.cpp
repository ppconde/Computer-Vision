#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


int main() {
    //presents user interface
    system("clear");        //clears terminal window
    cout
    << endl
    << " -------------------" << endl
    << "  Feature Detection " << endl
    << " -------------------" << endl
    << endl;

    Mat img_1 = imread("img/img1.jpg", IMREAD_GRAYSCALE );
    Mat img_2 = imread("img/img3.jpg", IMREAD_GRAYSCALE );

    if (!img_1.data || !img_2.data) {
        cout<< " --(!) Error reading images " << endl;
        return -1;
    }

    //detects keypoints and calculates descriptors
    int minHessian = 500;  //higher value = more "focused" keypoints

    Ptr<SURF> detector = SURF::create( minHessian );

    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    detector->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
    detector->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);

    //matches descriptors using FLANN matcher
    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //calculates distance between keypoints
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    cout << "--Max dist: " << max_dist << endl;
    cout << "--Min dist: " << min_dist << endl;

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    vector<DMatch> good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ ) {
        if (matches[i].distance <= max(2*min_dist, 0.02)) {
            good_matches.push_back( matches[i]);
        }
    }

    //draws good matches
    Mat img_matches;

    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //displays detected matches
    imshow( "Good Matches", img_matches );

    for (int i = 0; i < (int)good_matches.size(); i++) {
        cout << "-- Good Match [" << i << "] Keypoint 1: "
             << good_matches[i].queryIdx << "  -- Keypoint 2: "
             << good_matches[i].trainIdx << endl;
    }

    waitKey(0);
    return 0;
}
