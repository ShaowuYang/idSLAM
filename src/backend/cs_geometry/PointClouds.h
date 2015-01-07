#ifndef _CS_GEOMETRY_POINTCLOUDS_H_
#define _CS_GEOMETRY_POINTCLOUDS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sophus/se3.hpp>

#include "Camera.h"

namespace cs_geom {

class PointClouds {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const Camera& cam, const cv::Mat& depth);
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbdToPointCloud(const Camera& cam,
                                                                   const cv::Mat& rgb,
                                                                   const cv::Mat& depth,
                                                                   bool bgr = true,
                                                                   int step = 1);

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbdToPointCloud(const Camera& cam,
                                                                   const cv::Mat& rgb,
                                                                   const cv::Mat& depth,
                                                                   const Sophus::SE3d& se3,
                                                                   bool bgr = true,
                                                                   int step = 1);
};

} // namespace

#endif /* _CS_GEOMETRY_POINTCLOUDS_H_ */
