#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

using namespace pcl;

int  main (int argc, char** argv) {
	bool sel;
	char filename[100];

    //presents user interface
    system("clear");
    cout << endl << "-------------" << endl;
    cout << " ICP Aligner " << endl;
    cout << "-------------" << endl << endl;
    cout << " Choose given clouds [0] or user clouds [1]: ";
    cin >> sel;
    cout << endl;

    //creates new point cloud
    PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);

    //reads files
    if (sel) {
	    	if (io::loadPCDFile<PointXYZRGB> ("img/filt_kinect1.pcd", *cloud1) == -1) {
        	PCL_ERROR ("Couldn't read file.\n");
        	return (-1);
    	}
    	if (io::loadPCDFile<PointXYZRGB> ("img/filt_kinect2.pcd", *cloud2) == -1) {
        	PCL_ERROR ("Couldn't read file.\n");
        	return (-1);
    	}
    }
    else {
	    if (io::loadPCDFile<PointXYZRGB> ("img/filt_office1.pcd", *cloud1) == -1) {
	        PCL_ERROR ("Couldn't read file.\n");
	        return (-1);
	    }
	    if (io::loadPCDFile<PointXYZRGB> ("img/filt_office2.pcd", *cloud2) == -1) {
	        PCL_ERROR ("Couldn't read file.\n");
	        return (-1);
	    }
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

    //colors cloudAligned red for viewing purposes
    int p = 0;
    for (int i = 0; i < (*cloudAligned).height; ++i) {
        for (int j = 0; j < (*cloudAligned).width; j++) {
            (*cloudAligned).points[p].r = 255;
            (*cloudAligned).points[p].g = 0;
            (*cloudAligned).points[p].b = 0;

            p++;
        }
    }

    //displays point cloud (original and aligned)
    visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud1, "c1");
    viewer.showCloud(cloud2, "c2");
    viewer.showCloud(cloudAligned, "Aligned");
    while (!viewer.wasStopped());

    return (0);
}
