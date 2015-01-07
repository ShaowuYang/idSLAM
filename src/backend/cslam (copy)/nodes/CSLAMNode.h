#ifndef _CSLAM_NODE_H_
#define _CSLAM_NODE_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cs_ptam/Keyframe.h>
#include <cs_ptam/Edges.h>
#include <cs_geometry/Camera.h>
#include <slam/Keyframe.h>
#include <slam/SLAMSystem.h>

namespace cslam {

class CSLAMNode {
public:
    CSLAMNode(ros::NodeHandle& nh);

    void keyframeCb(const cs_ptam::KeyframeConstPtr& kf);
    void edgesCb(const cs_ptam::EdgesConstPtr& kf);


protected:
    boost::shared_ptr<Keyframe> messageToKeyframe(const cs_ptam::KeyframeConstPtr& msg);
    std::vector<boost::shared_ptr<Edge> > messageToEdges(const cs_ptam::EdgesConstPtr& msg);

    void publishTF(const ros::TimerEvent& e = ros::TimerEvent());
    void publishVis();

    boost::scoped_ptr<cs_geom::Camera> cam_;
    boost::scoped_ptr<SLAMSystem> slam_;

    std::string world_frame_, body_frame_, camera_frame_;

    bool use_ground_data_;
    bool close_loops_;

    double vis_pointcloud_interval_;
    int vis_pointcloud_step_;

    // ros infrastructure:
    ros::NodeHandle nh_, nh_private_;

    ros::Publisher vis_kf_pub_, vis_points_pub_, vis_pointcloud_pub_, vis_edges_pub_;
    ros::Subscriber kf_sub_, edges_sub_;

    // tf infrastructure
    ros::Timer tf_timer_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
};

}

#endif /* _CSLAM_NODE_H_ */
