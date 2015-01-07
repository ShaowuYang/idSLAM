#ifndef _CSLAM_SERIALIZE_SOPHUS_H_
#define _CSLAM_SERIALIZE_SOPHUS_H_

#include <sophus/se3.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(
        Archive & ar,
        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
        const unsigned int file_version
        )
{
    for(size_t i=0; i<t.size(); i++)
        ar & t.data()[i];
}

template<class Archive, typename _Scalar>
inline void serialize(Archive &ar, Eigen::Quaternion<_Scalar>& q, const unsigned int version)
{
    ar & q.w() & q.x() & q.y() & q.z();
}

namespace serialization {
template<class Archive, typename _Scalar>
void save(Archive & ar, const Sophus::SO3Group<_Scalar>& m, const unsigned int version)
{
    ar & m.unit_quaternion();
}

template<class Archive, typename _Scalar>
void load(Archive & ar, Sophus::SO3Group<_Scalar>& m, const unsigned int version)
{
    Eigen::Quaternion<_Scalar> q;
    ar & q;
    m.setQuaternion(q);
}

// TODO: no need to split this?
template<class Archive, typename _Scalar>
void save(Archive & ar, const Sophus::SE3Group<_Scalar>& m, const unsigned int version)
{
    ar & m.so3();
    ar & m.translation();
}

template<class Archive, typename _Scalar>
void load(Archive & ar, Sophus::SE3Group<_Scalar>& m, const unsigned int version)
{
    ar & m.so3();
    ar & m.translation();
}


}
}

BOOST_SERIALIZATION_SPLIT_FREE(Sophus::SO3Group<double>)
BOOST_SERIALIZATION_SPLIT_FREE(Sophus::SE3Group<double>)

#endif /* _CSLAM_SERIALIZE_SOPHUS_H_ */
