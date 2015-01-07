#include "CSLAMNode.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

#include <cs_geometry/Conversions.h>
#include <cs_geometry/PointClouds.h>

using namespace cv;
using namespace cs_geom;
using namespace cslam;
using namespace std;

CSLAMNode::CSLAMNode(ros::NodeHandle& nh) : nh_(nh), nh_private_("~")
{
    std::string calib_file, voc_file;

    if (!nh_private_.getParam("vocabulary_file", voc_file)) {
        cerr << "You sucker did not give me a vocabulary file." << endl;
        exit(EXIT_FAILURE);
    }

    if (!nh_private_.getParam("calib_file", calib_file)) {
        cerr << "You sucker did not give me a calib file." << endl;
        exit(EXIT_FAILURE);
    }


    nh_private_.param("body_frame",      body_frame_,    std::string("/base_link"));
    nh_private_.param("camera_frame",    camera_frame_,  std::string("/camera_rgb_optical_frame"));
    nh_private_.param("world_frame",     world_frame_,   std::string("/cslam_world"));

    nh_private_.param("use_ground_data", use_ground_data_, true);
    nh_private_.param("close_loops",     close_loops_, true);

    nh_private_.param("vis_pointcloud_interval", vis_pointcloud_interval_, 2.0);
    nh_private_.param("vis_pointcloud_step",     vis_pointcloud_step_,     5);

    world_frame_ = "/cslam_world";

    // setup published topics
    vis_kf_pub_         = nh_.advertise<visualization_msgs::Marker>("/cslam/keyframes", 1);
    vis_points_pub_     = nh_.advertise<visualization_msgs::Marker>("/cslam/points", 1);
    vis_edges_pub_      = nh_.advertise<visualization_msgs::Marker>("/cslam/edges", 1);
    vis_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cslam/pointcloud", 1);


    std::cout << "waiting for body to camera transform " << std::endl;
    tf::StampedTransform bodyTcamStamped;
    // need to wait for valid time for durations to make any sense (consider simulated time)
    ros::Time::waitForValid();
    tf_listener_.waitForTransform(body_frame_, camera_frame_, ros::Time::now(), ros::Duration(0.5));
    tf_listener_.lookupTransform(body_frame_, camera_frame_, ros::Time(0), bodyTcamStamped);


    cam_.reset(new Camera(calib_file));
    slam_.reset(new SLAMSystem(*cam_,voc_file, toSophusSE3(bodyTcamStamped), close_loops_));


    tf_timer_ = nh_.createTimer(ros::Duration(0.1), &CSLAMNode::publishTF, this);

    kf_sub_    = nh_.subscribe<cs_ptam::Keyframe>("/ptam/keyframes", 10, boost::bind(&CSLAMNode::keyframeCb, this, _1));
    edges_sub_ = nh_.subscribe<cs_ptam::Edges>("/ptam/edges", 10, boost::bind(&CSLAMNode::edgesCb, this, _1));
}

void CSLAMNode::keyframeCb(const cs_ptam::KeyframeConstPtr& msg)
{
    cout << "map points: " << msg->mappoints.size() << endl;
    cout << "corners: ";
    for (uint i = 0; i < msg->levels.size(); i++)
        cout << msg->levels[i].corners.size() << " ";
    cout << endl;

    boost::shared_ptr<Keyframe> kf = messageToKeyframe(msg);

    slam_->addKeyframe(kf);

    publishVis();
}

void CSLAMNode::edgesCb(const cs_ptam::EdgesConstPtr& msg)
{
    std::vector<boost::shared_ptr<Edge> > edges = messageToEdges(msg);
    slam_->addEdges(edges);

    publishVis();
}

boost::shared_ptr<Keyframe> CSLAMNode::messageToKeyframe(const cs_ptam::KeyframeConstPtr& msg)
{
    boost::shared_ptr<Keyframe> kf(new Keyframe());
    kf->id = msg->id;
    kf->time = TimeStamp(msg->kftime.sec, msg->kftime.nsec);

    kf->ptamPosewTc = toSophusSE3(msg->pose);
    kf->posewTc     = kf->ptamPosewTc;

    kf->haveImuData = msg->haveImuData;
    kf->imuNaviTc   = toSophusSO3(msg->naviTCam);

    kf->haveGroundData = (use_ground_data_ && msg->haveGroundData);
    kf->groundTc       = toSophusSE3(msg->groundTCam);

    // Copy depth image:
    kf->depthImage = cv::Mat(msg->depthImage.height, msg->depthImage.width, CV_16UC1,
                             (void*) &msg->depthImage.data[0], msg->depthImage.step).clone();

    // Copy rgb image:
    kf->rgbImage = cv::Mat(msg->rgbImage.height, msg->rgbImage.width, CV_8UC3,
                           (void*) &msg->rgbImage.data[0]).clone();

    // Copy levels:
    kf->levels.resize(msg->levels.size());
    for (uint l = 0; l < msg->levels.size(); l++) {
        int levelscale = 1 << l;
        const cs_ptam::Level& lmsg = msg->levels[l];
        Level& lkf = kf->levels[l];

        // Make sure image is actually copied:
        lkf.image = cv::Mat(lmsg.height, lmsg.width, CV_8UC1,
                            (void*) &lmsg.data[0], lmsg.step).clone();

        assert(lmsg.corners.size() > 0);
        lkf.corners.resize(lmsg.corners.size());
        lkf.cornerDepth.resize(lmsg.corners.size());
        for (uint c = 0; c < lmsg.corners.size(); c++) {
            lkf.corners[c].x = lmsg.corners[c].ix;
            lkf.corners[c].y = lmsg.corners[c].iy;
        }

        lkf.cornerDepth.resize(lmsg.cornerDepth.size());
        for (uint c = 0; c < lmsg.cornerDepth.size(); c++) {
            lkf.cornerDepth[c] = lmsg.cornerDepth[c];
        }
    }

    // Copy map points:
    Sophus::SE3d poseInv = kf->posewTc.inverse();
    kf->mapPoints.resize(msg->mappoints.size());
    for (uint i = 0; i < msg->mappoints.size(); i++) {
        const cs_ptam::Mappoint& mpmsg = msg->mappoints[i];
        MapPoint& mpkf = kf->mapPoints[i];

        mpkf.level = mpmsg.level;

        // p3d: PTAM stores global map points, we need relative map points:
        mpkf.p3d = poseInv*Eigen::Vector3d(mpmsg.px, mpmsg.py, mpmsg.pz);

        mpkf.pi.x  = mpmsg.ix;
        mpkf.pi.y  = mpmsg.iy;
    }

    return kf;
}

std::vector<boost::shared_ptr<Edge> > CSLAMNode::messageToEdges(const cs_ptam::EdgesConstPtr& msg)
{
    std::vector<boost::shared_ptr<Edge> > edges;

    edges.resize(msg->edges.size());

    for (uint e = 0; e < edges.size(); e++) {
        edges[e].reset(new Edge);
        const cs_ptam::Edge& emsg = msg->edges[e];

        edges[e]->idA   = emsg.idA;
        edges[e]->idB   = emsg.idB;
        edges[e]->aTb   = toSophusSE3(emsg.poseATB);
        edges[e]->type  = EDGE_PTAM;
        edges[e]->valid = true;
    }

    return edges;
}

void CSLAMNode::publishTF(const ros::TimerEvent&)
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(toTfTrans(slam_->cslamTptam().inverse()),
                                                       ros::Time::now(),
                                                       "/ptam_world",
                                                       world_frame_));
}

void CSLAMNode::publishVis()
{
    const std::vector<boost::shared_ptr<Keyframe> >& kfs = slam_->keyframes();

    if (vis_kf_pub_.getNumSubscribers() > 0) {
        visualization_msgs::Marker kf_marker;
        kf_marker.header.frame_id = world_frame_;
        kf_marker.header.stamp = ros::Time::now();
        kf_marker.ns = "map";
        kf_marker.id = 0;
        kf_marker.action = visualization_msgs::Marker::ADD;
        kf_marker.pose.position.x = 0;
        kf_marker.pose.position.y = 0;
        kf_marker.pose.position.z = 0;
        kf_marker.pose.orientation.x = 0.0;
        kf_marker.pose.orientation.y = 0.0;
        kf_marker.pose.orientation.z = 0.0;
        kf_marker.pose.orientation.w = 1.0;
        kf_marker.scale.x = 0.01;
        kf_marker.scale.y = 0.01;
        kf_marker.scale.z = 0.01;
        kf_marker.color.r = 165.0/255.0f;
        kf_marker.color.g = 30.0/255.0f;
        kf_marker.color.b = 55.0/255.0f;
        kf_marker.color.a = 1.0f;
        kf_marker.lifetime = ros::Duration();
        kf_marker.type = visualization_msgs::Marker::LINE_LIST;

        kf_marker.points.resize(kfs.size()*6);
        uint ip = 0;

        BOOST_FOREACH(const boost::shared_ptr<Keyframe> kf, kfs) {
            const Sophus::SE3d& pose = kf->posewTc;

            geometry_msgs::Point camPos = toGeomMsgPoint(pose.translation());

            Eigen::Vector3d dx(0.075, 0.,   0.);
            Eigen::Vector3d dy(0.,    0.05, 0.);
            Eigen::Vector3d dz(0.,    0.,   0.1);

            kf_marker.points[ip++] = camPos;
            kf_marker.points[ip++] = toGeomMsgPoint(pose*dx);

            kf_marker.points[ip++] = camPos;
            kf_marker.points[ip++] = toGeomMsgPoint(pose*dy);

            kf_marker.points[ip++] = camPos;
            kf_marker.points[ip++] = toGeomMsgPoint(pose*dz);
        }
        vis_kf_pub_.publish(kf_marker);
    } // end if vis_kf_pub_ has subscribers

    if (vis_points_pub_.getNumSubscribers() > 0) {
        visualization_msgs::Marker points_marker;

        points_marker.header.frame_id = world_frame_;
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "map";
        points_marker.id = 0;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.position.x = 0;
        points_marker.pose.position.y = 0;
        points_marker.pose.position.z = 0;
        points_marker.pose.orientation.x = 0.0;
        points_marker.pose.orientation.y = 0.0;
        points_marker.pose.orientation.z = 0.0;
        points_marker.pose.orientation.w = 1.0;
        points_marker.color.r = 0.0f;
        points_marker.color.g = 0.0f;
        points_marker.color.b = 0.0f;
        points_marker.color.a = 0.5f;
        points_marker.scale.x = 0.02;
        points_marker.scale.y = 0.02;
        points_marker.scale.z = 0.02;
        points_marker.lifetime = ros::Duration();
        points_marker.type = visualization_msgs::Marker::POINTS;

        BOOST_FOREACH(const boost::shared_ptr<Keyframe> kf, kfs) {
            const Sophus::SE3d& kfPose = kf->posewTc;
            BOOST_FOREACH(cslam::MapPoint p, kf->mapPoints) {
                Eigen::Vector3d pWorld = kfPose*p.p3d;
                unsigned char gray     = kf->levels[p.level].image.at<unsigned char>(p.pi);
                std_msgs::ColorRGBA color;
                color.a = 1.0;
                color.r = color.g = color.b = (1./255.)*gray;
                points_marker.colors.push_back(color);
                points_marker.points.push_back(toGeomMsgPoint(pWorld));
            }
        }

        vis_points_pub_.publish(points_marker);
    } // end if vis_points_pub_ has subscribers

    static ros::Time lastCloudPublished_ = ros::Time(0);
    if (vis_pointcloud_pub_.getNumSubscribers() > 0 &&
            (ros::Time::now() - lastCloudPublished_).toSec() > vis_pointcloud_interval_)
    {
        int step = vis_pointcloud_step_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        BOOST_FOREACH(const boost::shared_ptr<Keyframe> kf, kfs) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud = cs_geom::PointClouds::rgbdToPointCloud(
                        *cam_, kf->rgbImage, kf->depthImage, kf->posewTc, step);
            *fullCloud += *thisCloud;
        }

        sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*fullCloud, *msg);
        msg->header.frame_id = world_frame_;
        msg->header.stamp = ros::Time::now();

        vis_pointcloud_pub_.publish(msg);
        lastCloudPublished_ = ros::Time::now();
    }

    if (vis_edges_pub_.getNumSubscribers() > 0) {
        visualization_msgs::Marker edge_marker;

        edge_marker.header.frame_id = world_frame_;
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.ns = "map";
        edge_marker.id = 0;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.pose.position.x = 0;
        edge_marker.pose.position.y = 0;
        edge_marker.pose.position.z = 0;
        edge_marker.pose.orientation.x = 0.0;
        edge_marker.pose.orientation.y = 0.0;
        edge_marker.pose.orientation.z = 0.0;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.scale.x = 0.01;
        edge_marker.scale.y = 0.01;
        edge_marker.scale.z = 0.01;
        edge_marker.color.r = 165.0/255.0f;
        edge_marker.color.g = 30.0/255.0f;
        edge_marker.color.b = 55.0/255.0f;
        edge_marker.color.a = 1.0f;
        edge_marker.lifetime = ros::Duration();
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;

        BOOST_FOREACH(const boost::shared_ptr<Keyframe> kf, kfs) {
            const Sophus::SE3d& kfPose = kf->posewTc;

            Eigen::Vector3d from = kfPose.translation();

            for (Keyframe::EdgeMap::const_iterator it = kf->edges.begin();
                 it != kf->edges.end(); it++) {
                 const int idB = it->first;
                 const boost::shared_ptr<Edge> edge = it->second;
                 assert(idB == edge->idB);

                 if (idB >= (int) kfs.size())
                     continue;

                 Eigen::Vector3d to = kfs[idB]->posewTc.translation();
                 edge_marker.points.push_back(toGeomMsgPoint(from));
                 edge_marker.points.push_back(toGeomMsgPoint(to));

                 std_msgs::ColorRGBA color;
                 color.a = 1.0;
                 switch (edge->type) {
                 case EDGE_PTAM:
                     color.g = 1.0;
                     break;
                 case EDGE_MOTIONMODEL:
                     color.r = 1.0;
                     break;
                 case EDGE_LOOP:
                     color.b = 1.0;
                     break;
                 }

                 edge_marker.colors.push_back(color);
                 edge_marker.colors.push_back(color);
            }

            if (kf->haveGroundData) {
                std_msgs::ColorRGBA colorGround;
                colorGround.a = 0.7;
                colorGround.r = 1.0;
                colorGround.g = 0.5;

                Eigen::Vector3d ground = (kf->posewTc*kf->groundTc.inverse()).translation();

                edge_marker.points.push_back(toGeomMsgPoint(from));
                edge_marker.points.push_back(toGeomMsgPoint(ground));

                edge_marker.colors.push_back(colorGround);
                edge_marker.colors.push_back(colorGround);
            }
        }

        vis_edges_pub_.publish(
                    edge_marker);
    } // end if vis_edges_pub_ has subscribers
}
