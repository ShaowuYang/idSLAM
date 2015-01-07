#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <sophus/se3.hpp>

#include <cs_geometry/Camera.h>
#include <cs_geometry/FivePointSolver.h>
#include <cs_geometry/PointClouds.h>

#include <registration/Registrator5P.h>
#include <registration/RegistratorBAPTAM.h>
#include <tools/Profiler.h>
#include <slam/Keyframe.h>

using namespace cslam;
using namespace cs_geom;
using namespace std;
using namespace pcl;

double validDepth = 0.3;

int main(int argc, char **argv)
{
    Sophus::SE3d relPose1T0;

    if (argc!= 10) {
        cout << "usage: " << argv[0] << " camera.yaml kf0 kf1 relpose" << endl;
        return EXIT_FAILURE;
    }

    cs_geom::Camera cam(argv[1]);
    if (!cam.isGood()) {
        cerr << "Could not open camera: " << argv[1] << endl;
    }

    Profiler* prof = Profiler::Instance();
    string kf0file = argv[2];
    string kf1file = argv[3];

    Eigen::Matrix<double, 6, 1> log;
    for (int i = 0; i < 6; i++) {
        log[i] = boost::lexical_cast<double>(argv[i+4]);
    }
    Sophus::SE3d relPose = Sophus::SE3d::exp(log);

    Keyframe kf0, kf1;

    std::ifstream ifs(kf0file.c_str(), std::ios::binary);
    boost::archive::binary_iarchive ia0(ifs);
    ia0 >> kf0;
    ifs.close();

    ifs.open(kf1file.c_str(), std::ios::binary);
    boost::archive::binary_iarchive ia1(ifs);
    ia1 >> kf1;
    ifs.close();

    PointCloud<PointXYZRGB>::Ptr cloud0 = PointClouds::rgbdToPointCloud(cam, kf0.rgbImage, kf0.depthImage);
    PointCloud<PointXYZRGB>::Ptr cloud1 = PointClouds::rgbdToPointCloud(cam, kf1.rgbImage, kf1.depthImage, relPose);
    io::savePCDFile(string("cloud0")+string(".pcd"), *cloud0, true);
    io::savePCDFile(string("cloud1")+string(".pcd"), *cloud1, true);


    return 0;
}
