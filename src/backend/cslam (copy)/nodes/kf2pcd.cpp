#include <iostream>

#include <cs_geometry/Camera.h>
#include <cs_geometry/PointClouds.h>

#include <slam/Keyframe.h>

#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace cslam;
using namespace cs_geom;
using namespace pcl;
using namespace std;

PointCloud<PointXYZRGB>::Ptr mapPointsToCloud(const Keyframe& kf)
{
    PointCloud<PointXYZRGB>::Ptr res(new PointCloud<PointXYZRGB>());

    int nPoints = kf.mapPoints.size();

    res->width  = std::ceil(nPoints);
    res->height = std::ceil(1);
    res->resize(res->width*res->height);

    PointCloud<PointXYZRGB>::iterator pc_iter = res->begin();
    for (int i = 0; i < kf.mapPoints.size(); i++) {
        const MapPoint& mp = kf.mapPoints[i];
        PointXYZRGB& pt = *pc_iter++;

        pt.x = mp.p3d[0]; pt.y = mp.p3d[1]; pt.z = mp.p3d[2];
        pt.r = 255; pt.g = 0; pt.b = 0;
    }

    return res;
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        cout << "usage: " << argv[0] << " calib.yaml kf" << endl;
        return 1;
    }

    Camera cam(argv[1]);
    if (!cam.isGood()) {
        cout << "failed to open camera calibration: " << argv[1] << endl;
        return 1;
    }

    Keyframe kf;
    std::ifstream ifs(argv[2], std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    ia >> kf;
    ifs.close();

    PointCloud<PointXYZRGB>::Ptr cloud = PointClouds::rgbdToPointCloud(cam, kf.rgbImage, kf.depthImage);
    io::savePCDFile(string(argv[2])+string(".pcd"), *cloud, true);

    PointCloud<PointXYZRGB>::Ptr mpCloud = mapPointsToCloud(kf);
    io::savePCDFile(string(argv[2])+string("mp.pcd"), *mpCloud, true);

}
