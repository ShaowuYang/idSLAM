#include "PointClouds.h"

using namespace geometry;
using namespace cv;
using namespace pcl;
using namespace std;


PointCloud<PointXYZRGB>::Ptr PointClouds::rgbdToPointCloud(const ptam::CameraModel& cam,
                                                           const Mat& rgb,
                                                           const Mat& depth,
                                                           const TooN::SE3<>& se3,
                                                           bool bgr,
                                                           int step)
{
    assert(depth.type() == CV_16UC1);
    assert(depth.cols == cam.GetImageSize()[0]);
    assert(depth.rows == cam.GetImageSize()[1]);

    assert(rgb.cols == cam.GetImageSize()[0]);
    assert(rgb.rows == cam.GetImageSize()[1]);

    cv::Mat myrgb;
    switch(rgb.type()) {
    case CV_8UC1:
        cvtColor(rgb, myrgb, CV_GRAY2BGR);
        break;
    case CV_8UC3:
        myrgb = rgb;
        break;
    default:
        assert(false);
    }

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    PointCloud<PointXYZRGB>::Ptr res(new PointCloud<PointXYZRGB>());

    int widthstep = step;
    res->width  = std::ceil((double) depth.cols/ (double) widthstep);
    res->height = std::ceil((double) depth.rows/ (double) step);
    res->resize(res->width*res->height);

//    FileStorage fs("depthpc.txt", FileStorage::WRITE);
//    fs << "depth" << depth;

    PointCloud<PointXYZRGB>::iterator pc_iter = res->begin();
    for (unsigned int y = 0; y < depth.rows; y += step) {
        const uint16_t* depthPtr = depth.ptr<uint16_t>(y);
        const Vec3b* rgbPtr = myrgb.ptr<Vec3b>(y);
        for (unsigned int x = 0; x < depth.cols; x += widthstep) {
            const uint16_t& d = depthPtr[x];
            PointXYZRGB& pt = *pc_iter++;
            if ((d > 0) && (d < 5000)) {
                TooN::Vector<2> pixel = TooN::makeVector(x,y);
                TooN::Vector<3> p3d = TooN::unproject(cam.UnProjectSafe(pixel))*(d*1e-3);
                TooN::Vector<3> p = se3*p3d; // (cam.unprojectPixelTo3D(Eigen::Vector2d(x,y))*(d*1e-3));
                pt.x = p[0]; pt.y = p[1]; pt.z = p[2];

                const Vec3b& col = rgbPtr[x];
                if (bgr) {
                    pt.b = col[0];
                    pt.g = col[1];
                    pt.r = col[2];
                } else {
                    pt.r = col[0];
                    pt.g = col[1];
                    pt.b = col[2];
                }
            } else {
                pt.x = pt.y = pt.z = bad_point;
            }
        }
    }
//    fs.release();

    return res;
}

