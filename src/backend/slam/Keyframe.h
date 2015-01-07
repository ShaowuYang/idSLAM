#ifndef _CSLAM_KEYFRAME_H_
#define _CSLAM_KEYFRAME_H_

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>

#include "../tools/serialize/SerializeOpenCV.h"
#include "../tools/serialize/SerializeSophus.h"

namespace cslam {

enum EdgeType {EDGE_PTAM, EDGE_MOTIONMODEL, EDGE_LOOP, EDGE_INIT};

struct Edge
{
    Edge(int idA, int idB, EdgeType type, const Sophus::SE3d& aTb) :
        idA(idA), idB(idB), type(type), aTb(aTb), valid(true), dSceneDepthMean(1.0) {}
    Edge() : valid(false) {}
    int idA;
    int idB;
    Sophus::SE3d aTb;

    EdgeType type;

    bool valid;

    double dSceneDepthMean;      // Hacky hueristics to improve the information matrix of edges in pose graph

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & idA & idB & aTb & type & valid;
    }
};

struct MapPoint
{
    MapPoint(): repeated(false) {}

    Eigen::Vector3d p3d;
    cv::Point2i pi;

    double depth;
    int level;

    bool repeated; // is it a repeated map point to previous map point in older kf?

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & p3d & pi & depth & level;
    }
};

struct Level
{
    cv::Mat image;
    std::vector<cv::Point2i> corners;
    std::vector<float> cornerDepth;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & image & corners & cornerDepth;
    }
};

struct TimeStamp
{
    TimeStamp() : sec(0), nsec(0) {}
    TimeStamp(int sec, int nsec) : sec(sec), nsec(nsec) {}
    int sec, nsec;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & sec & nsec;
    }
};

struct Keyframe
{
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & id & time;
        ar & ptamPosewTc;
        ar & posewTc;

        ar & haveImuData & imuNaviTc;
        ar & haveGroundData & groundTc;
        ar & levels;
        ar & mapPoints & mpKeypoints & mpDesc;

        ar & keypoints & kpDepth & kpDesc;
        ar & edges;

        ar & depthImage & rgbImage;
    }

    int id; /// incremental Keyframe id (from PTAM).
    TimeStamp time; /// timestamp of original camera image (from PTAM).

    Sophus::SE3d ptamPosewTc; /// PTAM's pose estimate for this keyframe, stored for reference (from PTAM).
    Sophus::SE3d posewTc; /// CSLAM's pose estimate, as refined using PGO.

    bool haveImuData;             /// Did we get an IMU orientation for this Keyframe?
    Sophus::SO3d imuNaviTc;       /// IMU orientation, if available: ^Navi R_C, disregard yaw orientation.

    bool haveGroundData;          /// Did we find a ground plane for this Keyframe?
    Sophus::SE3d groundTc;        /// Transform ^G T_C

    std::vector<Level> levels; /// PTAM's levels structure (from PTAM).
    std::vector<MapPoint> mapPoints; /// PTAM's map points (from PTAM).
    std::vector<cv::KeyPoint> mpKeypoints;
    cv::Mat mpDesc;             /// Map point descriptors


    typedef std::map<int, boost::shared_ptr<Edge> > EdgeMap;
    EdgeMap edges; /// Outgoing edges to other keyframes
    std::multimap<double, int> neighbor_ids_ordered_by_distance;

    std::vector<cv::KeyPoint> keypoints; /// Keypoints computed based on PTAM's corners (not all corners become keypoints).
    std::vector<float> kpDepth; /// Depth values at keypoint locations or 0 if unavailable.
    cv::Mat kpDesc; /// Keypoint descriptors.

    cv::Mat depthImage; /// Depth image - if available (from PTAM).
    cv::Mat rgbImage; /// RGB image - if available (from PTAM).

    double dSceneDepthMean;      // Hacky hueristics to improve the information matrix of edges in pose graph

    void finalizeKeyframe(); /// Finalize CSLAM keyframe based on data provided by PTAM.

    void saveKeyframe() const;
    void saveKeyframe(const std::string& name) const;
};

} // namespace

#endif /* _CSLAM_KEYFRAME_H_ */
