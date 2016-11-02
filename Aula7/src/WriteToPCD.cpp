#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;

int main (int argc, char** argv) {
    PointCloud<PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    io::savePCDFileASCII ("test_pcd.pcd", cloud);
    cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << endl;

    for (size_t i = 0; i < cloud.points.size (); ++i) {
        cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << endl;
    }

  return (0);
}
