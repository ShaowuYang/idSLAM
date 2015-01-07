#ifndef _CSLAM_SERIALIZE_OPENCV_H_
#define _CSLAM_SERIALIZE_OPENCV_H_

#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)

namespace boost {
    template <class Archive, typename Scalar>
    inline void serialize(Archive& ar, cv::Point_<Scalar>& p, const unsigned int version) {
        ar & p.x & p.y;
    }

    template<class Archive>
    inline void serialize(Archive& ar, cv::KeyPoint& kp, const unsigned int version) {
        ar & kp.angle & kp.class_id & kp.octave & kp.pt & kp.response & kp.size;
    }

    namespace serialization {

    template<class Archive>
    void save(Archive & ar, const ::cv::Mat& m, const unsigned int version)
    {
        size_t elem_size = m.elemSize();
        size_t elem_type = m.type();

        ar & m.cols;
        ar & m.rows;
        ar & elem_size;
        ar & elem_type;

        const size_t data_size = m.cols * m.rows * elem_size;
        ar & boost::serialization::make_array(m.ptr(), data_size);
    }

    template<class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
        int cols, rows;
        size_t elem_size, elem_type;

        ar & cols;
        ar & rows;
        ar & elem_size;
        ar & elem_type;

        m.create(rows, cols, elem_type);

        size_t data_size = m.cols * m.rows * elem_size;
        ar & boost::serialization::make_array(m.ptr(), data_size);
    }

    }
}

#endif /* _CSLAM_SERIALIZE_OPENCV_H_ */
