#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace cv;

int  main (int argc, char** argv) {
    char filename[100];
    bool sel;
    int file;

    //presents user interface
    system("clear");
    cout << endl << "--------------------" << endl;
    cout << " Point Cloud Viewer " << endl;
    cout << "--------------------" << endl << endl;
    cout << " Choose to read XML file containing points' coordinates [0] or kinect point cloud [1]: ";
    cin >> sel;
    cout << endl;

    if (sel) {
        //creates new point clouds
        PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

        cout << " Choose point cloud file [1 or 2] or input file name [3]: ";
        cin >> file;
        cout << endl;

        //reads files
        if (file == 3) {
            char name[20];
            cout << " Input the point cloud file's name: ";
            cin >> name;
            cout << endl;

            sprintf(filename, "img/%s", name);
        }
        else {
            sprintf(filename, "img/office%d.pcd", file);
        }
        if (io::loadPCDFile<PointXYZRGB> (filename, *cloud) == -1) {
            PCL_ERROR ("Couldn't read file.\n");
            return (-1);
        }

        //filtering to reduce number of points
        PointCloud<PointXYZRGB>::Ptr filtCloud (new PointCloud<PointXYZRGB>);
        
        VoxelGrid<PointXYZRGB> vox;
        vox.setInputCloud(cloud);
        vox.setLeafSize(0.05f, 0.05f, 0.05f);
        vox.filter(*filtCloud);

        //displays point cloud
        visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(filtCloud);
        while (!viewer.wasStopped());
    }
    else {
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
                    //(*cloud).points[p].r = image.at<Vec3b>(i, j)val[0];
                    //(*cloud).points[p].g = image.at<Vec3b>(i, j)val[1];
                    //(*cloud).points[p].b = image.at<Vec3b>(i, j)val[2];
                }

                p++;
            }
        }

        //displays point cloud
        visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloud);
        while (!viewer.wasStopped());
    }
 
    return (0);
}