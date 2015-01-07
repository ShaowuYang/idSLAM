#ifndef _CONVERSIONS_H_
#define _CONVERSIONS_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <LinearMath/btTransform.h>

#include <TooN/so3.h>
#include <TooN/se3.h>

namespace idSLAM{
tf::Transform se3_to_btTrans(const TooN::SE3<>& se3);
tf::Quaternion so3_to_quat(const TooN::SO3<>& so3);
void quat_from_so3(geometry_msgs::Quaternion& quat, const TooN::SO3<>& so3);

geometry_msgs::Pose toGeomMsgPose(const TooN::SE3<>& se3);
geometry_msgs::Point toGeomMsgPoint(const TooN::Vector<3>& v3);
TooN::SO3<> toToonSO3(const tf::Quaternion& q);

geometry_msgs::Point geometry_msgs_point(const TooN::Vector<3>& v3);
} // namespace

#endif /* _CONVERSIONS_H_ */
