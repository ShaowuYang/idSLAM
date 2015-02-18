#ifndef _MAPVISUALIZATION_H_
#define _MAPVISUALIZATION_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <cv.h>
#include "ptam/CameraModel.h"

namespace ptam{
class CameraModel;
class Map;
class Tracker;
}

namespace idSLAM{

class MapVisualization {
public:
	MapVisualization();
    void publishMapVisualization(const ptam::Map* map, const ptam::Tracker* tracker, const ros::Publisher &camera_pub, const ros::Publisher &point_pub, const std::string& world_frame, const ros::Publisher &camera_pubsec, const bool dualcamera=false);
    void publishlandingpad(const ptam::Tracker* tracker, const ros::Publisher &landingpad_pub);
    void publishPointCloud(const ptam::Map* map, const ptam::Tracker* tracker, const ros::Publisher &vis_pointcloud_pub, const std::string& world_frame, double vis_publish_interval_, int vis_pointcloud_step_, double cellSize = 0.02);
    void publishCrtPointCloud(const ptam::Map* map, const ptam::Tracker* tracker, const ros::Publisher &vis_pointcloud_pub, const ros::Publisher &vis_pointcloud_pubsec,
                              const std::string& world_frame, double vis_publish_interval_ = 0.1);

    void renderDebugImage(cv::Mat& rgb_cv, const ptam::Tracker* tracker, const ptam::Map* map); // camera may not be const as it precaches data
    void renderDebugImageSec(cv::Mat& rgb_cv, const ptam::Tracker* tracker, const ptam::Map* map); // camera may not be const as it precaches data
    void renderRefImage(cv::Mat& rgb_cv, const ptam::Tracker* tracker); // camera may not be const as it precaches data
    void writeFrame(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_img_msg, int id);
	
private:
    std::auto_ptr<ptam::CameraModel> camera;
    std::auto_ptr<ptam::CameraModel> camerasec[AddCamNumber];
};

} // namespace


#endif /* _MAPVISUALIZATION_H_ */
