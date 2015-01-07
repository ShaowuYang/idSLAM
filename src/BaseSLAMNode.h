#include <fstream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <TooN/se3.h>

#include <idSLAM/LoadMap.h>
#include <idSLAM/SaveMap.h>
#include <idSLAM/SetMappingEnabled.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace ptam{
class Map;
class MapMaker;
class Tracker;
}
namespace cslam{
class SLAMSystem;
class backend;
}
namespace cs_geom{
class Camera;
}

namespace idSLAM{
class MapVisualization;
// Base class for ROS Nodes for parallel tracking and mapping (PTAM)
class BaseSLAMNode : public nodelet::Nodelet{
//class BaseSLAMNode{
public:
    BaseSLAMNode();
    ~BaseSLAMNode();
protected:
    virtual void onInit();
    void InitBackend();
    void publishPose(ros::Time stamp = ros::Time::now());
    void postProcessFrame();

    bool LoadMap(idSLAM::LoadMap::Request &req,
                 idSLAM::LoadMap::Response &resp);
    bool SetMappingEnabled(idSLAM::SetMappingEnabled::Request &req,
                           idSLAM::SetMappingEnabled::Response &resp);
    bool SaveMap(idSLAM::SaveMap::Request &req,
                 idSLAM::SaveMap::Response &resp);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // PTAM objects
    boost::scoped_ptr<ptam::Map> map_;
    boost::scoped_ptr<ptam::MapMaker> map_maker_;
    boost::scoped_ptr<ptam::Tracker> tracker_;
    boost::scoped_ptr<MapVisualization> map_viz_;

    //backend objects
    boost::scoped_ptr<cs_geom::Camera> cam_;
    boost::scoped_ptr<cslam::SLAMSystem> ss_;
    boost::scoped_ptr<cslam::backend> backend_;

    // ROS Publishers
    ros::Publisher cam_marker_pub_;
    ros::Publisher point_marker_pub_;
    ros::Publisher cam_pose_pub_;
    ros::Publisher cam_transform_pub_;
    ros::Publisher robot_to_world_trans_pub_;
    ros::Publisher vis_pointcloud_pub_;

    // ROS Service Servers
    ros::ServiceServer mapping_serv_;
    ros::ServiceServer load_map_serv_;
    ros::ServiceServer save_map_serv_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    // options
    std::string world_frame_;

    bool show_debug_image_;
    bool write_keyframes_;
    bool write_pos_;
    std::string map_frame_;
    std::string settings_file_;

    double vis_publish_interval_;
    int vis_pointcloud_step_;
    double cellSize;

    // state
    double frame_stamp_; // time stamp of processed frame, needs to be set by derived class
    std::ofstream pos_stream_;
    TooN::SE3<> camPose4pub;// Tcw for pubish

    bool isdualcam; // using multiple cameras? Abusing "dual" in this project
    bool rgbIsBgr_; // are we working with bgr images?
};
} // namespace
