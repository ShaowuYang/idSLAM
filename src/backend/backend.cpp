#include "backend.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>

#include <cs_geometry/Conversions.h>
#include <cs_geometry/PointClouds.h>
#include <ptam/MapPoint.h>

using namespace cv;
using namespace cs_geom;
using namespace backend;
using namespace std;

LoopClosing::LoopClosing(ros::NodeHandle &nh, ros::NodeHandle &nh_pri, SLAMSystem &ss) :
    nh_(nh),
    nh_private_(nh_pri),
    slam_(ss)
{
    nh_private_.param("world_frame",     world_frame_,   std::string("/ptam_world"));

    pubmap_ = slam_.pubmap_;
    // setup published topics
    vis_kf_pub_         = nh_.advertise<visualization_msgs::Marker>("/backend/keyframes", 1);
    vis_points_pub_     = nh_.advertise<visualization_msgs::Marker>("/backend/points", 1);
    vis_edges_pub_      = nh_.advertise<visualization_msgs::Marker>("/backend/edges", 1);
    vis_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/backend/pointcloud", 1);

    backinfolog.open("backend.log");
    cout <<"logfile in backend open?: " << backinfolog.is_open() << endl;
    if (backinfolog.is_open()){
        backinfolog.setf(std::ios::fixed, std::ios::floatfield);
        backinfolog.precision(10);
    }

    start();// This CVD::thread func starts the backend thread with function run()
}

void LoopClosing::StopThread(){
    this->stop();
    this->join();
}

void LoopClosing::run(){
    while(!shouldStop()){
        //should do CHECK_RESET
        // check the waiting list of the front end, and add kfs, edges
        {
            boost::mutex::scoped_lock lock(slam_.mutex_wl);
            while(slam_.wlKeyFrames.empty()){
                kfWlEmpty_.wait(lock);
            }
        }
        if(slam_.wlKeyFrames.size() > 0 ){
            ros::Time backbegin = ros::Time::now();

            boost::unique_lock<boost::mutex> lockg(slam_.syncMutex);// write access
            slam_.setGMapUpdated(false);
            lockg.unlock();
            cout << "adding edges and kfs sizes: " << slam_.wlEdges.size() << ", "
                    << slam_.wlKeyFrames.size() << endl;

            // add edges first, then kfs, since we need to add a outgoing edge of a kf
            slam_.addEdges();
            slam_.addKeyframes();// kfs to be added after edges

            ros::Time backkftime = ros::Time::now();
            double backmidtime = backkftime.toSec() - backbegin.toSec();

            slam_.runPGO();

            ros::Time backendtime = ros::Time::now();
            double backpgotime = backendtime.toSec() - backkftime.toSec();

            if(pubmap_)
                publishVis();

            backinfolog << backbegin.toSec() << " "
                        << backmidtime << " "
                        << backpgotime << " ";
            //and the keyframe size and map point size in the global map, and kfsize in pgo
            backinfolog << slam_.getKfSize() << " "
                        << slam_.getMapPointSize() << " "
                        << slam_.kfinpgo << " "
                        << slam_.getCornersSize() << "\n";;
        }
        sleep(5);
    }
}

void LoopClosing::addKeyframe(const int kf_id)
{
//    boost::shared_ptr<Keyframe> kf( new Keyframe);
//    *kf = kf_id;
    slam_.wlKeyFrames.push_back(kf_id);
    kfWlEmpty_.notify_one();
}

void LoopClosing::addEdges(const std::vector<ptam::Edge> msg)
{
    std::vector<boost::shared_ptr<ptam::Edge> > edges = messageToEdges(msg);
//    slam_.addEdges(edges);

    for (int i = 0; i < edges.size(); i++)
        slam_.wlEdges.push_back(edges[i]);
}

bool LoopClosing::relocaliseRegister(const boost::shared_ptr<ptam::KeyFrame> goodkf, const boost::shared_ptr<ptam::KeyFrame> kf, Sophus::SE3d &result, double minInliers)
{
    if ( slam_.relocaliseRegister(goodkf, kf, result, minInliers))
        return true;
    else
        return false;
}

std::vector<boost::shared_ptr<ptam::Edge> > LoopClosing::messageToEdges(const std::vector<ptam::Edge>& msg)
{
    std::vector<boost::shared_ptr<ptam::Edge> > edges;

    edges.resize(msg.size());

    for (uint e = 0; e < edges.size(); e++) {
        edges[e].reset(new ptam::Edge);
        const ptam::Edge& emsg = msg[e];

        edges[e]->idA   = emsg.idA;
        edges[e]->idB   = emsg.idB;
        edges[e]->aTb   = emsg.aTb;
        edges[e]->type  = ptam::EDGE_PTAM;
        edges[e]->valid = true;
    }

    return edges;
}

void LoopClosing::publishTF(const ros::TimerEvent&)
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(toTfTrans(slam_.cslamTptam().inverse()),
                                                       ros::Time::now(),
                                                       "/ptam_world",
                                                       world_frame_));
}

void LoopClosing::publishVis()
{
    const std::vector<boost::shared_ptr<ptam::KeyFrame> >& kfs = slam_.keyframes();
    bool useDifWorldFrame = false; // use a different world frame for rviz visualization

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

        BOOST_FOREACH(const boost::shared_ptr<ptam::KeyFrame> kf, kfs) {
            TooN::SE3<> kfpose = kf->se3CfromW.inverse();
            if (useDifWorldFrame){
                TooN::Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                             -1.0, 0, 0, // a world frame which pointing downward.
                                             0, -1.0, 0);
                TooN::SO3<> rot = TooN::SO3<>();
                rot = datam;
                kfpose = rot * kfpose;
            }
            const Sophus::SE3d& pose = cs_geom::toSophusSE3(kfpose);

            geometry_msgs::Point camPos = toGeomMsgPoint(pose.translation());

            Eigen::Vector3d dx(0.1, 0.,   0.);
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

    /*if (vis_points_pub_.getNumSubscribers() > 0) {
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

        BOOST_FOREACH(const boost::shared_ptr<ptam::KeyFrame> kf, kfs) {
            const Sophus::SE3d& kfPose = cs_geom::toSophusSE3(kf->se3CfromW.inverse());
            BOOST_FOREACH(boost::shared_ptr<ptam::MapPoint> mp, kf->mapPoints) {
                Eigen::Vector3d pWorld = kfPose*cs_geom::toEigenVec(mp->v3WorldPos);
                unsigned char gray     = kf->aLevels[mp->nSourceLevel].im[mp->irCenter];
                std_msgs::ColorRGBA color;
                color.a = 1.0;
                color.r = color.g = color.b = (1./255.)*gray;
                points_marker.colors.push_back(color);
                points_marker.points.push_back(toGeomMsgPoint(pWorld));
            }
        }

        vis_points_pub_.publish(points_marker);
    } */// end if vis_points_pub_ has subscribers

    static ros::Time lastCloudPublished_ = ros::Time(0);
//    if (vis_pointcloud_pub_.getNumSubscribers() > 0 &&
//            (ros::Time::now() - lastCloudPublished_).toSec() > vis_pointcloud_interval_)
//    {
//        int step = vis_pointcloud_step_;

//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//        BOOST_FOREACH(const boost::shared_ptr<Keyframe> kf, kfs) {
//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud = cs_geom::PointClouds::rgbdToPointCloud(
//                        cam_, kf->rgbImage, kf->depthImage, kf->posewTc, step);
//            *fullCloud += *thisCloud;
//        }

//        sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2());
//        pcl::toROSMsg(*fullCloud, *msg);
//        msg->header.frame_id = world_frame_;
//        msg->header.stamp = ros::Time::now();

//        vis_pointcloud_pub_.publish(msg);
//        lastCloudPublished_ = ros::Time::now();
//    }

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
        edge_marker.color.a = 0.7f;
        edge_marker.lifetime = ros::Duration();
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;

        BOOST_FOREACH(const boost::shared_ptr<ptam::KeyFrame> kf, kfs) {
            TooN::SE3<> kfpose = kf->se3CfromW.inverse();
            if (useDifWorldFrame){
                TooN::Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                             -1.0, 0, 0, // a world frame which pointing downward.
                                             0, -1.0, 0);
                TooN::SO3<> rot = TooN::SO3<>();
                rot = datam;
                kfpose = rot * kfpose;
            }
            const Sophus::SE3d& kfPose = cs_geom::toSophusSE3(kfpose);

            Eigen::Vector3d from = kfPose.translation();

            for (ptam::KeyFrame::EdgeMap::const_iterator it = kf->edges.begin();
                 it != kf->edges.end(); it++) {
                 const int idB = it->first;
                 const boost::shared_ptr<ptam::Edge> edge = it->second;
                 assert(idB == edge->idA);

                 if (idB >= (int) kfs.size())
                     continue;

                 TooN::SE3<> kfpose = kfs[idB]->se3CfromW.inverse();
                 if (useDifWorldFrame){
                     TooN::Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                                  -1.0, 0, 0, // a world frame which pointing downward.
                                                  0, -1.0, 0);
                     TooN::SO3<> rot = TooN::SO3<>();
                     rot = datam;
                     kfpose = rot * kfpose;
                 }
                 Eigen::Vector3d to = cs_geom::toSophusSE3(kfpose).translation();
                 edge_marker.points.push_back(toGeomMsgPoint(from));
                 edge_marker.points.push_back(toGeomMsgPoint(to));

                 std_msgs::ColorRGBA color;
                 color.a = 0.6;
                 color.r = 0.4;
                 color.g = 0.4;
                 color.b = 0.4;
                 switch (edge->type) {
                 case ptam::EDGE_PTAM:
                     color.g = 1.0;
                     break;
                 case ptam::EDGE_MOTIONMODEL:
                     color.b = 1.0;
                     break;
                 case ptam::EDGE_LOOP:
                     color.r = 1.0;
                     color.a = 1.0;
                     color.g = 0.;
                     color.b = 0.;
                     break;
                 }

                 edge_marker.colors.push_back(color);
                 edge_marker.colors.push_back(color);
            }

            /*if (kf->haveGroundData) {
                std_msgs::ColorRGBA colorGround;
                colorGround.a = 0.7;
                colorGround.r = 1.0;
                colorGround.g = 0.5;

                Eigen::Vector3d ground = (kf->posewTc*kf->groundTc.inverse()).translation();

                edge_marker.points.push_back(toGeomMsgPoint(from));
                edge_marker.points.push_back(toGeomMsgPoint(ground));

                edge_marker.colors.push_back(colorGround);
                edge_marker.colors.push_back(colorGround);
            }*/
        }

        vis_edges_pub_.publish(
                    edge_marker);
    } // end if vis_edges_pub_ has subscribers
}
