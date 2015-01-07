#include "PointClouds.h"

using namespace cs_geom;
using namespace cv;
using namespace pcl;
using namespace std;

PointCloud<PointXYZ>::Ptr PointClouds::depthToPointCloud(const Camera& cam,
                                                         const Mat& depth)
{
    assert(depth.type() == CV_16UC1);
    assert(depth.cols == cam.width());
    assert(depth.rows == cam.height());

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    PointCloud<PointXYZ>::Ptr res(new PointCloud<PointXYZ>());

    res->width = depth.cols;
    res->height = depth.rows;
    res->resize(depth.cols*depth.rows);

    PointCloud<PointXYZ>::iterator pc_iter = res->begin();
    for (unsigned int y = 0; y < depth.rows; y++) {
        const uint16_t* rowPtr = depth.ptr<uint16_t>(y);
        for (unsigned int x = 0; x < depth.cols; x++) {
            const uint16_t& d = rowPtr[x];
            PointXYZ& pt = *pc_iter++;

            if (d > 0) {
                cv::Point3f p = cam.unprojectPixelTo3D(cv::Point2i(x,y))*(d*1e-3);
                pt.x = p.x; pt.y = p.y; pt.z = p.z;
            } else {
                pt.x = pt.y = pt.z = bad_point;
            }
        }
    }

    return res;
}

PointCloud<PointXYZRGB>::Ptr PointClouds::rgbdToPointCloud(const Camera& cam,
                                                           const Mat& rgb,
                                                           const Mat& depth,
                                                           bool bgr,
                                                           int step)
{
    assert(depth.type() == CV_16UC1);
    assert(depth.cols == cam.width());
    assert(depth.rows == cam.height());

    assert(rgb.cols == cam.width());
    assert(rgb.rows == cam.height());

    cv::Mat myrgb;
    switch(rgb.type()) {
    case CV_8UC1:
        cvtColor(rgb, myrgb, CV_GRAY2BGR);
        bgr = true;
        break;
    case CV_8UC3:
        myrgb = rgb;
        break;
    default:
        assert(false);
    }

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    PointCloud<PointXYZRGB>::Ptr res(new PointCloud<PointXYZRGB>());

    res->width  = std::ceil((double) depth.cols/ (double) step);
    res->height = std::ceil((double) depth.rows/ (double) step);
    res->resize(depth.cols*depth.rows);

    PointCloud<PointXYZRGB>::iterator pc_iter = res->begin();
    for (unsigned int y = 0; y < depth.rows; y += step) {
        const uint16_t* depthPtr = depth.ptr<uint16_t>(y);
        const Vec3b* rgbPtr = myrgb.ptr<Vec3b>(y);
        for (unsigned int x = 0; x < depth.cols; x += step) {
            const uint16_t& d = depthPtr[x];
            PointXYZRGB& pt = *pc_iter++;

            if (d > 0) {
                cv::Point3f p = cam.unprojectPixelTo3D(cv::Point2i(x,y))*(d*1e-3);
                pt.x = p.x; pt.y = p.y; pt.z = p.z;

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

    return res;
}

PointCloud<PointXYZRGB>::Ptr PointClouds::rgbdToPointCloud(const Camera& cam,
                                                           const Mat& rgb,
                                                           const Mat& depth,
                                                           const Sophus::SE3d& se3,
                                                           bool bgr,
                                                           int step)
{
    assert(depth.type() == CV_16UC1);
    assert(depth.cols == cam.width());
    assert(depth.rows == cam.height());

    assert(rgb.cols == cam.width());
    assert(rgb.rows == cam.height());

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

    res->width  = std::ceil((double) depth.cols/ (double) step);
    res->height = std::ceil((double) depth.rows/ (double) step);
    res->resize(res->width*res->height);

    PointCloud<PointXYZRGB>::iterator pc_iter = res->begin();
    for (unsigned int y = 0; y < depth.rows; y += step) {
        const uint16_t* depthPtr = depth.ptr<uint16_t>(y);
        const Vec3b* rgbPtr = myrgb.ptr<Vec3b>(y);
        for (unsigned int x = 0; x < depth.cols; x += step) {
            const uint16_t& d = depthPtr[x];
            PointXYZRGB& pt = *pc_iter++;

            if (d > 0) {
                Eigen::Vector3d p = se3*(cam.unprojectPixelTo3D(Eigen::Vector2d(x,y))*(d*1e-3));
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

    return res;
}

