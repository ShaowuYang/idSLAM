#ifndef _CSLAM_NODE_H_
#define _CSLAM_NODE_H_

#include <cvd/thread.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <idSLAM/Keyframe.h>
#include <idSLAM/Edges.h>
#include <cs_geometry/Camera.h>
#include <slam/Keyframe.h>
#include <slam/SLAMSystem.h>
//#include <slam/>

#include <boost/thread/condition.hpp>

namespace cslam {

class backend : protected CVD::Thread{
public:
    backend(ros::NodeHandle& nh, cs_geom::Camera &cam, cslam::SLAMSystem &ss);

    void addKeyframe(const Keyframe kf);
    void addPoints(const Keyframe kf);
    void addEdges(const std::vector<Edge> edges);
    void updateKfPoses(const vector<boost::shared_ptr<Keyframe> > kfs);

    virtual void StopThread();

protected:
    virtual void run();

    boost::condition kfWlEmpty_;
    boost::shared_ptr<Keyframe> messageToKeyframe(const idSLAM::KeyframeConstPtr& msg);
    std::vector<boost::shared_ptr<Edge> > messageToEdges(const std::vector<cslam::Edge>& msg);

    // TODO: consider to move to baseslamnode
    void publishTF(const ros::TimerEvent& e = ros::TimerEvent());
    void publishVis();
    ofstream backinfolog;

//    Map &mMap;
//    boost::scoped_ptr<cs_geom::Camera> cam_;
//    boost::scoped_ptr<SLAMSystem> slam_;
    cs_geom::Camera &cam_;
    SLAMSystem &slam_;// the back end slam system, read and write access

    std::string world_frame_, body_frame_, camera_frame_;

    bool use_ground_data_;
    bool close_loops_;
    bool pubmap_;

    double vis_pointcloud_interval_;
    int vis_pointcloud_step_;

    // ros infrastructure:
    ros::NodeHandle nh_, nh_private_;

    ros::Publisher vis_kf_pub_, vis_points_pub_, vis_pointcloud_pub_, vis_edges_pub_;
//    ros::Subscriber kf_sub_, edges_sub_;

    // tf infrastructure
    ros::Timer tf_timer_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
};

}

#endif /* _CSLAM_NODE_H_ */
