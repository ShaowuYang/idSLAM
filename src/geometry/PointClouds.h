#ifndef _CS_GEOMETRY_POINTCLOUDS_H_
#define _CS_GEOMETRY_POINTCLOUDS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <TooN/se3.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "ptam/CameraModel.h"

namespace geometry {

class PointClouds {
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbdToPointCloud(const ptam::CameraModel& cam,
                                                                   const cv::Mat& rgb,
                                                                   const cv::Mat& depth,
                                                                   const TooN::SE3<>& se3, // world from camera
                                                                   bool bgr = true,
                                                                   int step = 1);
};

} // namespace

#endif /* _CS_GEOMETRY_POINTCLOUDS_H_ */
