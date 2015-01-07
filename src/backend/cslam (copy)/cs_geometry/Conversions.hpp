#include "Conversions.h"

#include <Eigen/Geometry>

namespace cs_geom {

tf::Transform toTfTrans(const TooN::SE3<>& se3)
{
    tf::Transform transf;
    transf.setRotation(toTfQuat(se3.get_rotation()));
    const TooN::Vector<3>& transl = se3.get_translation();
    transf.setOrigin(tf::Vector3(transl[0],
                                 transl[1],
                                 transl[2]));

    return transf;
}

tf::Transform toTfTrans(const Sophus::SE3d& se3)
{
    tf::Transform transf;
    transf.setRotation(toTfQuat(se3.so3().unit_quaternion()));

    const Eigen::Vector3d& t = se3.translation();
    transf.setOrigin(tf::Vector3(t[0], t[1], t[2]));

    return transf;
}

tf::Quaternion toTfQuat(const TooN::SO3<>& so3)
{
    TooN::Vector<3> rotVec = so3.ln();
    tf::Vector3 axis(rotVec[0], rotVec[1], rotVec[2]);
    double angle = axis.length();

    tf::Quaternion q;
    if(angle > 0)
        q = tf::Quaternion(axis.normalized(), angle); // this will crash if angle == 0
    else
        q = tf::Quaternion(0, 0, 0, 1); // identity

    return q;
}

tf::Quaternion toTfQuat(const Eigen::Quaterniond& q)
{
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

tf::Matrix3x3 toTfMat(const Eigen::Matrix3d& eigenMat)
{
    tf::Matrix3x3 mat;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mat[i][j] = eigenMat(i,j);

    return mat;
}

TooN::SO3<> toToonSO3(const tf::Quaternion& q)
{
    assert(fabs(q.length() - 1.0) < 1e-6);

    double angle = q.w()*2.;
    double s = sqrt(1. - q.w()*q.w());
    TooN::Vector<3> axis;
    axis[0] = q.x(); axis[1] = q.y(); axis[2] = q.z();

    if (s < 1e-6)  { // cannot normalize
        axis[0] = 1.0; axis[1] = 0.; axis[2] = 0.;
    } else {
        axis /= s;
    }

    TooN::SO3<> so3(axis*angle);
    return so3;
}

TooN::SE3<> toToonSE3(const Sophus::SE3d& sophus)
{
    Sophus::SE3d::Tangent logS = sophus.log();

    TooN::Vector<6> logT;

    for (int i = 0; i < 6; i++)
        logT[i] = logS[i];

    return TooN::SE3<>::exp(logT);
}


Sophus::SO3d toSophusSO3(const geometry_msgs::Quaternion& q)
{
    Sophus::SO3d so3;

    so3.setQuaternion(Eigen::Quaterniond(q.w, q.x, q.y, q.z));

    return so3;
}

Sophus::SE3d toSophusSE3(const TooN::SE3<>& toon)
{
    TooN::Vector<6> logT = toon.ln();
    Sophus::SE3d::Tangent logS;

    for (int i = 0; i < 6; i++)
        logS[i] = logT[i];

    return Sophus::SE3d::exp(logS);
}

Sophus::SE3d toSophusSE3(const geometry_msgs::Pose& msg)
{
    Sophus::SE3d se3;

    se3.so3() = toSophusSO3(msg.orientation);
    se3.translation() = Eigen::Vector3d(msg.position.x,
                                        msg.position.y,
                                        msg.position.z);

    return se3;
}

Sophus::SE3d toSophusSE3(const tf::Transform& tf)
{
    Sophus::SE3d se3;

    const tf::Quaternion q = tf.getRotation();
    se3.so3().setQuaternion(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

    const tf::Vector3& t = tf.getOrigin();
    se3.translation() = Eigen::Vector3d(t[0], t[1], t[2]);

    return se3;
}

Eigen::Matrix3d toEigenMat(const tf::Matrix3x3& tfMat)
{
    Eigen::Matrix3d mat;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mat(i,j) = tfMat[i][j];

    return mat;
}

geometry_msgs::Quaternion toGeomMsgQuat(const TooN::SO3<>& so3) {
    tf::Quaternion quat = toTfQuat(so3);

    geometry_msgs::Quaternion q;
    q.x = quat.x();
    q.y = quat.y();
    q.z = quat.z();
    q.w = quat.w();
    return q;
}

geometry_msgs::Pose toGeomMsgPose(const TooN::SE3<>& se3)
{
    geometry_msgs::Pose pose;

    tf::Quaternion q = toTfQuat(se3.get_rotation());
    pose.position.x = se3.get_translation()[0];
    pose.position.y = se3.get_translation()[1];
    pose.position.z = se3.get_translation()[2];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

geometry_msgs::Point toGeomMsgPoint(const TooN::Vector<3>& v3)
{
    geometry_msgs::Point point;
    point.x = v3[0];
    point.y = v3[1];
    point.z = v3[2];
    return point;
}

geometry_msgs::Point toGeomMsgPoint(const Eigen::Vector3d& p)
{
    geometry_msgs::Point gp;
    gp.x = p[0];
    gp.y = p[1];
    gp.z = p[2];
    return gp;
}

Sophus::SO3d toSO3(const tf::Matrix3x3& m) {
    Sophus::SO3d so3;

    tf::Quaternion qTF;
    m.getRotation(qTF);

    so3.setQuaternion(Eigen::Quaterniond (qTF.w(), qTF.x(), qTF.y(), qTF.z()));
    return so3;
}

Sophus::SO3d toSO3(const tf::Quaternion& q) {
    Sophus::SO3d so3;

    so3.setQuaternion(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

    return so3;
}

}
