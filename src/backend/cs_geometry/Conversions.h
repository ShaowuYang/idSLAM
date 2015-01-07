#ifndef _CS_GEOMETRY_CONVERSIONS_H_
#define _CS_GEOMETRY_CONVERSIONS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <TooN/so3.h>
#include <TooN/se3.h>

#include <tf/LinearMath/Transform.h>

namespace cs_geom {

tf::Transform toTfTrans(const TooN::SE3<>& se3);
tf::Transform toTfTrans(const Sophus::SE3d& se3);

tf::Quaternion toTfQuat(const TooN::SO3<>& so3);
tf::Quaternion toTfQuat(const Eigen::Quaterniond& q);

tf::Matrix3x3 toTfMat(const Eigen::Matrix3d& eigenMat);

TooN::SO3<> toToonSO3(const tf::Quaternion& q);
TooN::SE3<> toToonSE3(const Sophus::SE3d& sophus);

Sophus::SO3d toSophusSO3(const geometry_msgs::Quaternion& q);

Sophus::SE3d toSophusSE3(const TooN::SE3<>& toon);
Sophus::SE3d toSophusSE3(const geometry_msgs::Pose& msg);
Sophus::SE3d toSophusSE3(const tf::Transform& msg);

Eigen::Matrix3d toEigenMat(const tf::Matrix3x3& tfMat);

geometry_msgs::Quaternion toGeomMsgQuat(const TooN::SO3<>& so3);
geometry_msgs::Pose toGeomMsgPose(const TooN::SE3<>& se3);
geometry_msgs::Point toGeomMsgPoint(const TooN::Vector<3>& v3);
geometry_msgs::Point toGeomMsgPoint(const Eigen::Vector3d& p);

Sophus::SO3d toSO3(const tf::Matrix3x3& m);
Sophus::SO3d toSO3(const tf::Quaternion& q);

}

#include "Conversions.hpp"

#endif /* _CS_GEOMETRY_CONVERSIONS_H_ */
