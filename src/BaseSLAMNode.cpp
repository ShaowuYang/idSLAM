#include "BaseSLAMNode.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <gvars3/instances.h>

#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/MapPoint.h"
#include "ptam/Tracker.h"
#include "ptam/TrackerData.h"

#include "conversions/conversions.h"
#include "MapVisualization.h"
using namespace ptam;
using namespace idSLAM;
using namespace backend;

BaseSLAMNode::BaseSLAMNode()
//    : nh_private_("~")
    : Nodelet()
{}

void BaseSLAMNode::onInit()
{
    nh_ = getNodeHandle();
    nh_private_  = ros::NodeHandle(getName().c_str());

    rgbIsBgr_ = false;

    if (!nh_private_.getParam("usingdualslam",isdualcam))
        isdualcam = false;///////////////dual camera case
    if (!nh_private_.getParam("show_debug_image",show_debug_image_))
        show_debug_image_ = false;
    if (!nh_private_.getParam("write_keyframes",write_keyframes_))
        write_keyframes_ = false;
    if (!nh_private_.getParam("write_pos",write_pos_))
        write_pos_ = false;
    if(!nh_private_.getParam("map_frame", map_frame_))
        map_frame_ = "map";
    if(!nh_private_.getParam("settings_file", settings_file_))
        settings_file_ = "settings.cfg";
    nh_private_.param("world_frame",        world_frame_,       std::string("/ptam_world"));

    GVars3::GUI.LoadFile(settings_file_);
    std::cout << "done loading settings file: " << settings_file_ << std::endl;

    // allow ROS to overwrite some parameters
    std::string calib_type, calib_file;
    if (nh_private_.getParam("camera_type",calib_type))
        GVars3::GV3::set_var("Camera.Type", std::string("\"") + calib_type + "\"");
    if (nh_private_.getParam("camera_file",calib_file))
        GVars3::GV3::set_var("Camera.File", std::string("\"") + calib_file + "\"");
    if (nh_private_.getParam("camerasec_type",calib_type))
        GVars3::GV3::set_var("Camerasec.Type", std::string("\"") + calib_type + "\"");
    if (nh_private_.getParam("camerasec_file",calib_file))
        GVars3::GV3::set_var("Camerasec.File", std::string("\"") + calib_file + "\"");

    map_.reset(new Map);
    InitBackend();// setup slam system, and init the backend
    map_maker_.reset(new MapMaker(*map_, *ss_, *backend_));

    tracker_.reset(new Tracker(*map_, *map_maker_));
    map_viz_.reset(new MapVisualization());

    // setup published topics
    point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ptam/points", 1);
    cam_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ptam/cameras", 1);
    cam_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ptam/pose",1);
    cam_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/ptam/transform",1);
    robot_to_world_trans_pub_= nh_.advertise<geometry_msgs::TransformStamped>("/ptam/robot_to_world",1);
    vis_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ptam/pointcloud2", 1);

    // setup provided services
    mapping_serv_ = nh_.advertiseService("set_mapping_enabled",&BaseSLAMNode::SetMappingEnabled,this);
    load_map_serv_ = nh_.advertiseService("load_map",&BaseSLAMNode::LoadMap,this);
    save_map_serv_ = nh_.advertiseService("save_map",&BaseSLAMNode::SaveMap,this);
}

BaseSLAMNode::~BaseSLAMNode()
{
    map_maker_.reset(); // blocks until mapmaker thread is done
}

void BaseSLAMNode::InitBackend()
{
    std::string calib_file, voc_file;

    if (!nh_private_.getParam("vocabulary_file", voc_file)) {
        cerr << "You did not give me a vocabulary file!!" << endl;
        exit(EXIT_FAILURE);
    }

    if (!nh_private_.getParam("calib_file", calib_file)) {
        cerr << "You did not give me a calib file." << endl;
        exit(EXIT_FAILURE);
    }

//    nh_private_.param("body_frame",      body_frame_,    std::string("/base_link"));
//    nh_private_.param("camera_frame",    camera_frame_,  std::string("/camera_rgb_optical_frame"));

    bool saveKeyframes, close_loops_;
    bool pubmap_;
    nh_private_.param("save_keyframes",  saveKeyframes, false);
    nh_private_.param("close_loops",     close_loops_, true);
    nh_private_.param("pub_backend_map", pubmap_, true);

//    world_frame_ = "/idslam_world";

    cam_.reset(new cs_geom::Camera(calib_file));
    ss_.reset(new SLAMSystem(*map_,*cam_,voc_file, close_loops_, saveKeyframes, pubmap_));

    //ini the backend
    backend_.reset(new backend::LoopClosing(nh_, nh_private_, *ss_));
}

void BaseSLAMNode::publishPose(ros::Time stamp)
{
    SE3<> camPose = camPose4pub.inverse();//tracker_->GetCurrentPose().inverse();

    if (cam_pose_pub_.getNumSubscribers() > 0 || write_pos_) {
        geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped);
        pose->header.stamp = stamp;
        pose->header.frame_id = "/ptam_world";
        pose->pose.position.x = camPose.get_translation()[0];
        pose->pose.position.y = camPose.get_translation()[1];
        pose->pose.position.z = camPose.get_translation()[2];
        quat_from_so3(pose->pose.orientation, camPose.get_rotation());
        cam_pose_pub_.publish(pose);

        if (write_pos_) {
            pos_stream_ << frame_stamp_ << ", "
                       << pose->pose.position.x <<  ", "
                       << pose->pose.position.y <<  ", "
                       << pose->pose.position.z <<  std::endl;
        }
    }

    if (cam_transform_pub_.getNumSubscribers() > 0) {
        geometry_msgs::TransformStampedPtr trans(new geometry_msgs::TransformStamped);
        trans->header.stamp = stamp;
        trans->header.frame_id = "/ptam_world";
        trans->transform.translation.x = camPose.get_translation()[0];
        trans->transform.translation.y = camPose.get_translation()[1];
        trans->transform.translation.z = camPose.get_translation()[2];
        quat_from_so3(trans->transform.rotation, camPose.get_rotation());
        cam_transform_pub_.publish(trans);
    }

    tf::Transform pcam_to_pworld = se3_to_btTrans(camPose);
//    tf_broadcaster_.sendTransform(tf::StampedTransform(pcam_to_pworld,
//                                                       ros::Time::now(),
//                                                       "/ptam_world",
//                                                       "/ptam_camera"));

    if (robot_to_world_trans_pub_.getNumSubscribers() > 0)
    {
        tf::StampedTransform robot_to_world, robot_to_pcam, pworld_to_world;
        try {
            tf_listener_.lookupTransform("/base_link","/ptam_camera",ros::Time(0),robot_to_pcam);
            tf_listener_.lookupTransform("/ptam_world","/world",ros::Time(0),pworld_to_world);
            robot_to_world.setData(robot_to_pcam
                                   * pcam_to_pworld
                                   * pworld_to_world);

            geometry_msgs::TransformStampedPtr trans(new geometry_msgs::TransformStamped);
            trans->header.frame_id = "/world";
            trans->header.stamp = ros::Time(0);

            const tf::Quaternion& q = robot_to_world.getRotation();
            trans->transform.rotation.x = q.x();
            trans->transform.rotation.y = q.y();
            trans->transform.rotation.z = q.z();
            trans->transform.rotation.w = q.w();
            trans->transform.translation.x = robot_to_world.getOrigin().x();
            trans->transform.translation.y = robot_to_world.getOrigin().y();
            trans->transform.translation.z = robot_to_world.getOrigin().z();
            robot_to_world_trans_pub_.publish(trans);
        } catch (tf::LookupException ex) {
            ROS_ERROR("Error looking up transforms: %s",ex.what());
        }
    }


}

void BaseSLAMNode::postProcessFrame()
{

}

bool BaseSLAMNode::SetMappingEnabled(idSLAM::SetMappingEnabled::Request &req,
                                     idSLAM::SetMappingEnabled::Response &resp)
{
    if (req.enabled) {
        std::cout << "enabling mapping..." << std::endl;
        map_maker_->EnableMapping();
        if (map_maker_->GetMappingEnabled())
            resp.success = true;
    } else {
        std::cout << "disabling mapping..." << std::endl;
        map_maker_->DisableMapping();
        if (!map_maker_->GetMappingEnabled())
            resp.success = false;
    }
    return true;
}


bool BaseSLAMNode::LoadMap(idSLAM::LoadMap::Request &req,
                           idSLAM::LoadMap::Response &resp)
{
    resp.success = map_->LoadMap(req.path);
    return true;
}


bool BaseSLAMNode::SaveMap(idSLAM::SaveMap::Request &req,
                           idSLAM::SaveMap::Response &resp)
{
    resp.success = map_->SaveMap(req.path);
    return true;
}
// Register this plugin with pluginlib.  Names must match nodelet_...xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(idSLAM, BaseSLAMNode,
                        BaseSLAMNode, nodelet::Nodelet);
