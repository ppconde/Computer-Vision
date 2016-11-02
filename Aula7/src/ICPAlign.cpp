#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

using namespace pcl;

int  main (int argc, char** argv) {
    //creates new point cloud
    PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

    //presents user interface
    system("clear");
    cout << endl << "-------------" << endl;
    cout << " ICP Aligner " << endl;
    cout << "-------------" << endl << endl;

    //creates new point cloud
    PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);

    //reads files
    if (io::loadPCDFile<PointXYZRGB> ("img/filt_office1.pcd", *cloud1) == -1) {
        PCL_ERROR ("Couldn't read file.\n");
        return (-1);
    }
    if (io::loadPCDFile<PointXYZRGB> ("img/filt_office2.pcd", *cloud2) == -1) {
        PCL_ERROR ("Couldn't read file.\n");
        return (-1);
    }

    //applies ICP alignment
    PointCloud<PointXYZRGB>::Ptr cloudAligned(new PointCloud<PointXYZRGB>);

    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    icp.setTransformationEpsilon(1e-6);
	icp.setMaxCorrespondenceDistance(0.25);
	icp.setMaximumIterations(50);
    icp.setInputCloud(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(*cloudAligned);

    //displays point cloud (original and aligned)
    visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud1);
    viewer.showCloud(cloudAligned);
    while (!viewer.wasStopped());

    return (0);
}