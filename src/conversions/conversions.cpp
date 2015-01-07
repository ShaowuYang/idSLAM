#include "conversions.h"

using namespace TooN;

namespace idSLAM{

tf::Transform se3_to_btTrans(const TooN::SE3<>& se3)
{
    tf::Transform transf;
    transf.setRotation(so3_to_quat(se3.get_rotation()));
    const Vector<3>& transl = se3.get_translation();
    transf.setOrigin(tf::Vector3(transl[0],
                               transl[1],
                               transl[2]));

    return transf;
}

tf::Quaternion so3_to_quat(const TooN::SO3<>& so3)
{
    double x, y, z, w;
    const Matrix<3,3> a = so3.get_matrix();
    double trace = a[0][0] + a[1][1] + a[2][2]; // I removed + 1.0f; see discussion with Ethan
    if( trace > 0 ) {// I changed M_EPSILON to 0
        double s = 0.5f / sqrtf(trace+ 1.0f);
        w = 0.25f / s;
        x = ( a[2][1] - a[1][2] ) * s;
        y = ( a[0][2] - a[2][0] ) * s;
        z = ( a[1][0] - a[0][1] ) * s;
    } else {
        if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
            double s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
            w = (a[2][1] - a[1][2] ) / s;
            x = 0.25f * s;
            y = (a[0][1] + a[1][0] ) / s;
            z = (a[0][2] + a[2][0] ) / s;
        } else if (a[1][1] > a[2][2]) {
            double s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
            w = (a[0][2] - a[2][0] ) / s;
            x = (a[0][1] + a[1][0] ) / s;
            y = 0.25f * s;
            z = (a[1][2] + a[2][1] ) / s;
        } else {
            double s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
            w = (a[1][0] - a[0][1] ) / s;
            x = (a[0][2] + a[2][0] ) / s;
            y = (a[1][2] + a[2][1] ) / s;
            z = 0.25f * s;
        }
    }
    return tf::Quaternion(x,y,z,w);
}


// code from:
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
void quat_from_so3(geometry_msgs::Quaternion& q, const TooN::SO3<>& so3) {
    const Matrix<3,3> a = so3.get_matrix();
    double trace = a[0][0] + a[1][1] + a[2][2]; // I removed + 1.0f; see discussion with Ethan
    if( trace > 0 ) {// I changed M_EPSILON to 0
        float s = 0.5f / sqrtf(trace+ 1.0f);
        q.w = 0.25f / s;
        q.x = ( a[2][1] - a[1][2] ) * s;
        q.y = ( a[0][2] - a[2][0] ) * s;
        q.z = ( a[1][0] - a[0][1] ) * s;
    } else {
        if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
            float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
            q.w = (a[2][1] - a[1][2] ) / s;
            q.x = 0.25f * s;
            q.y = (a[0][1] + a[1][0] ) / s;
            q.z = (a[0][2] + a[2][0] ) / s;
        } else if (a[1][1] > a[2][2]) {
            float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
            q.w = (a[0][2] - a[2][0] ) / s;
            q.x = (a[0][1] + a[1][0] ) / s;
            q.y = 0.25f * s;
            q.z = (a[1][2] + a[2][1] ) / s;
        } else {
            float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
            q.w = (a[1][0] - a[0][1] ) / s;
            q.x = (a[0][2] + a[2][0] ) / s;
            q.y = (a[1][2] + a[2][1] ) / s;
            q.z = 0.25f * s;
        }
    }
}

geometry_msgs::Point geometry_msgs_point(const TooN::Vector<3>& v3)
{
    geometry_msgs::Point point;
    point.x = v3[0];
    point.y = v3[1];
    point.z = v3[2];
    return point;
}

geometry_msgs::Pose toGeomMsgPose(const TooN::SE3<>& se3)
{
    geometry_msgs::Pose pose;

    tf::Quaternion q = so3_to_quat(se3.get_rotation());
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

}
