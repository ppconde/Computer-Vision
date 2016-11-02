#include <iostream>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace cv;

int  main (int argc, char** argv) {
    int p = 0;
    Mat pos3D;

    //imports points coordinates from external XML file
    FileStorage fs("3Dpoints.xml", FileStorage::READ);
    if(!fs.isOpened()){
        cerr << "Failed to open 3Dpoints.xml" << endl;
        return 1;
    }
    fs["coordinates"] >> pos3D;

    //creates new point cloud
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    //configures point cloud
    (*cloud).width    = pos3D.cols;
    (*cloud).height   = pos3D.rows;
    (*cloud).is_dense = false;
    (*cloud).points.resize ((*cloud).width * (*cloud).height);

    //reads points coordinates and populates cloud with them
    for (int i = 0; i < (*cloud).height; ++i) {
        for (int j = 0; j < (*cloud).width; j++) {
            //filters points with negative ZZ coordinate
            if (pos3D.at<Vec3f>(i, j)[2] > 0) {
                (*cloud).points[p].x = pos3D.at<Vec3f>(i, j)[0];
                (*cloud).points[p].y = pos3D.at<Vec3f>(i, j)[1];
                (*cloud).points[p].z = pos3D.at<Vec3f>(i, j)[2];
            }

            p++;
        }
    }

    //displays point cloud
    visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
  
    while (!viewer.wasStopped());
 
    return (0);
}