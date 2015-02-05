#include "MapVisualization.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>

#include <highgui.h>

#include "conversions/conversions.h"
#include <cvd/image_convert.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ptam/CameraModel.h"
#include "ptam/KeyFrame.h"
#include "ptam/Map.h"
#include "ptam/MapPoint.h"
#include "ptam/Tracker.h"
#include "ptam/TrackerData.h"

#include <geometry/PointClouds.h>

using namespace idSLAM;
using namespace ptam;

MapVisualization::MapVisualization()
    :camera(CameraModel::CreateCamera())
{
    for (int i = 0; i < AddCamNumber; i ++){
        std::auto_ptr<CameraModel> camera_temp (CameraModel::CreateCamera(i + 1));
        camerasec[i] = camera_temp;
    }
}

void MapVisualization::publishlandingpad(const Tracker *tracker, const ros::Publisher &landingpad_pub)
{
    static visualization_msgs::Marker landingpad_marker;
    static visualization_msgs::Marker landingpadcenter_marker;
    static visualization_msgs::Marker landingpadcorner_marker;

    static visualization_msgs::Marker inilandingpadcamerapose_marker;
    static visualization_msgs::Marker finishlandingpadcamerapose_marker;

    landingpad_marker.header.frame_id = "/ptam_world";
    landingpad_marker.header.stamp = ros::Time::now();
    landingpad_marker.ns = "landingpad";
    landingpad_marker.id = 0;
    landingpad_marker.action = visualization_msgs::Marker::ADD;
    landingpad_marker.pose.position.x = 0;
    landingpad_marker.pose.position.y = 0;
    landingpad_marker.pose.position.z = 0;
    landingpad_marker.pose.orientation.x = 0.0;
    landingpad_marker.pose.orientation.y = 0.0;
    landingpad_marker.pose.orientation.z = 0.0;
    landingpad_marker.pose.orientation.w = 1.0;
    landingpad_marker.color.r = 1.0f;
    landingpad_marker.color.g = 0.0f;
    landingpad_marker.color.b = 0.0f;
    landingpad_marker.color.a = 1.0f;
    landingpad_marker.scale.x = 0.02;
    landingpad_marker.scale.y = 0.02;
    landingpad_marker.scale.z = 0.02;
    landingpad_marker.lifetime = ros::Duration();
    landingpad_marker.type = visualization_msgs::Marker::LINE_LIST;

    landingpadcenter_marker.header.frame_id = "/ptam_world";
    landingpadcenter_marker.header.stamp = ros::Time::now();
    landingpadcenter_marker.ns = "landingpad";
    landingpadcenter_marker.id = 1;
    landingpadcenter_marker.action = visualization_msgs::Marker::ADD;
    landingpadcenter_marker.pose.position.x = tracker->mPadCenterWorld[0];
    landingpadcenter_marker.pose.position.y = tracker->mPadCenterWorld[1];
    landingpadcenter_marker.pose.position.z = tracker->mPadCenterWorld[2];
    landingpadcenter_marker.pose.orientation.x = 0.0;
    landingpadcenter_marker.pose.orientation.y = 0.0;
    landingpadcenter_marker.pose.orientation.z = 0.0;
    landingpadcenter_marker.pose.orientation.w = 1.0;
    landingpadcenter_marker.color.r = 1.0;
    landingpadcenter_marker.color.a = 1.0;
    landingpadcenter_marker.scale.x = 0.02;
    landingpadcenter_marker.scale.y = 0.02;
    landingpadcenter_marker.scale.z = 0.02;
    landingpadcenter_marker.lifetime = ros::Duration();
    landingpadcenter_marker.type = visualization_msgs::Marker::CUBE;

    inilandingpadcamerapose_marker.header.frame_id = "/ptam_world";
    inilandingpadcamerapose_marker.header.stamp = ros::Time::now();
    inilandingpadcamerapose_marker.ns = "landingpad";
    inilandingpadcamerapose_marker.id = 3;
    inilandingpadcamerapose_marker.action = visualization_msgs::Marker::ADD;
    inilandingpadcamerapose_marker.pose.position.x = tracker->iniPadCameraPoseWorld[0];
    inilandingpadcamerapose_marker.pose.position.y = tracker->iniPadCameraPoseWorld[1];
    inilandingpadcamerapose_marker.pose.position.z = tracker->iniPadCameraPoseWorld[2];
    inilandingpadcamerapose_marker.pose.orientation.x = 0.0;
    inilandingpadcamerapose_marker.pose.orientation.y = 0.0;
    inilandingpadcamerapose_marker.pose.orientation.z = 0.0;
    inilandingpadcamerapose_marker.pose.orientation.w = 1.0;
    inilandingpadcamerapose_marker.color.r = 1.0;
    inilandingpadcamerapose_marker.color.a = 1.0;
    inilandingpadcamerapose_marker.scale.x = 0.15;
    inilandingpadcamerapose_marker.scale.y = 0.15;
    inilandingpadcamerapose_marker.scale.z = 0.15;
    inilandingpadcamerapose_marker.lifetime = ros::Duration();
    inilandingpadcamerapose_marker.text = "P_1";
    inilandingpadcamerapose_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    finishlandingpadcamerapose_marker.header.frame_id = "/ptam_world";
    finishlandingpadcamerapose_marker.header.stamp = ros::Time::now();
    finishlandingpadcamerapose_marker.ns = "landingpad";
    finishlandingpadcamerapose_marker.id = 4;
    finishlandingpadcamerapose_marker.action = visualization_msgs::Marker::ADD;
    finishlandingpadcamerapose_marker.pose.position.x = tracker->finishPadCameraPoseWorld[0];
    finishlandingpadcamerapose_marker.pose.position.y = tracker->finishPadCameraPoseWorld[1];
    finishlandingpadcamerapose_marker.pose.position.z = tracker->finishPadCameraPoseWorld[2];
    finishlandingpadcamerapose_marker.pose.orientation.x = 0.0;
    finishlandingpadcamerapose_marker.pose.orientation.y = 0.0;
    finishlandingpadcamerapose_marker.pose.orientation.z = 0.0;
    finishlandingpadcamerapose_marker.pose.orientation.w = 1.0;
    finishlandingpadcamerapose_marker.color.r = 1.0;
    finishlandingpadcamerapose_marker.color.a = 1.0;
    finishlandingpadcamerapose_marker.scale.x = 0.15;
    finishlandingpadcamerapose_marker.scale.y = 0.15;
    finishlandingpadcamerapose_marker.scale.z = 0.15;
    finishlandingpadcamerapose_marker.lifetime = ros::Duration();
    finishlandingpadcamerapose_marker.text = "P_2";
    finishlandingpadcamerapose_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    landingpadcorner_marker.header.frame_id = "/ptam_world";
    landingpadcorner_marker.header.stamp = ros::Time::now();
    landingpadcorner_marker.ns = "landingpad";
    landingpadcorner_marker.id = 2;
    landingpadcorner_marker.action = visualization_msgs::Marker::ADD;
    landingpadcorner_marker.color.b = 1.0;
    landingpadcorner_marker.color.a = 1.0;
    landingpadcorner_marker.scale.x = 0.02;
    landingpadcorner_marker.scale.y = 0.02;
    landingpadcorner_marker.scale.z = 0.02;
    landingpadcorner_marker.lifetime = ros::Duration();
    landingpadcorner_marker.type = visualization_msgs::Marker::CUBE_LIST;

    // read access: shared lock
    landingpad_marker.points.resize(8);
    landingpad_marker.colors.resize(8);
    landingpadcorner_marker.points.resize(4);
    landingpadcorner_marker.colors.resize(4);
    Vector<3> T;
    geometry_msgs::Point corner;
    cv::Vec3b color;
    int ii = 0;
    for (unsigned int i = 0; i < 3; i++) {
        switch(i) {
        case 0: color = cv::Vec3b(255, 0, 0); break;
        case 1: color = cv::Vec3b(0, 255, 0); break;
        case 2: color = cv::Vec3b(0, 0, 255); break;
        }
        landingpadcorner_marker.colors[i].b = color[0]/255;
        landingpadcorner_marker.colors[i].g = color[1]/255;
        landingpadcorner_marker.colors[i].r = color[2]/255;

        landingpad_marker.colors[ii].b = 55.0/255.0f;//color[0]/255;
        landingpad_marker.colors[ii].g = 30.0/255.0f;//color[1]/255;
        landingpad_marker.colors[ii].r = 165.0/255.0f;//color[2]/255;

        T = tracker->mPadCornersWorld[i];
        corner = geometry_msgs_point(T);
        landingpad_marker.points[ii++] = corner;

        landingpadcorner_marker.points[i] = corner;

        landingpad_marker.colors[ii].b = 55.0/255.0f;//color[0]/255;
        landingpad_marker.colors[ii].g = 30.0/255.0f;//color[1]/255;
        landingpad_marker.colors[ii].r = 165.0/255.0f;//color[2]/255;

        T = tracker->mPadCornersWorld[i+1];
        corner = geometry_msgs_point(T);
        landingpad_marker.points[ii++] = corner;



    }
    color = cv::Vec3b(255, 255, 0);
    landingpadcorner_marker.colors[3].b = color[0]/255;
    landingpadcorner_marker.colors[3].g = color[1]/255;
    landingpadcorner_marker.colors[3].r = color[2]/255;

    landingpad_marker.colors[ii].b = 55.0/255.0f;//color[0]/255;
    landingpad_marker.colors[ii].g = 30.0/255.0f;//color[1]/255;
    landingpad_marker.colors[ii].r = 165.0/255.0f;//color[2]/255;

    T = tracker->mPadCornersWorld[3];
    corner = geometry_msgs_point(T);
    landingpad_marker.points[ii++] = corner;
    landingpadcorner_marker.points[3] = corner;

    landingpad_marker.colors[ii].b = 55.0/255.0f;//color[0]/255;
    landingpad_marker.colors[ii].g = 30.0/255.0f;//color[1]/255;
    landingpad_marker.colors[ii].r = 165.0/255.0f;//color[2]/255;

    T = tracker->mPadCornersWorld[0];
    corner = geometry_msgs_point(T);
    landingpad_marker.points[ii] = corner;

    landingpad_pub.publish(landingpad_marker);
    landingpad_pub.publish(landingpadcenter_marker);
    landingpad_pub.publish(landingpadcorner_marker);
    if (tracker->isLandingpadPoseGet&&!tracker->isFinishPadDetection)
        landingpad_pub.publish(inilandingpadcamerapose_marker);
    if (tracker->isFinishPadDetection)
        landingpad_pub.publish(finishlandingpadcamerapose_marker);
}

// @param cellSize control integrating points in point cloud among different keyframes
void MapVisualization::publishPointCloud(const Map* map, const Tracker* tracker, const ros::Publisher &vis_pointcloud_pub, const std::string& world_frame,
                                         double vis_publish_interval_, int vis_pointcloud_step_,
                                         double cellSize)
{
    static ros::Time lastCloudPublished_ = ros::Time(0);
    if (vis_pointcloud_pub.getNumSubscribers() > 0 &&
            (ros::Time::now() - lastCloudPublished_).toSec() >= vis_publish_interval_)
    {
        int step = vis_pointcloud_step_;

        unsigned int nCams = map->vpKeyFrames.size();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud_long(new pcl::PointCloud<pcl::PointXYZRGB>());
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        static unsigned int nfullcloud = 0;// simply ignoring the dynamic changes of pointcloud during SLAM

        if (map->didFullBA)
            for (unsigned int i = 0; i < nCams; i += step) {
                cv::Mat rgb(map->vpKeyFrames[i]->rgbImage.size().y, map->vpKeyFrames[i]->rgbImage.size().x, CV_8UC3, map->vpKeyFrames[i]->rgbImage.data());
                cv::Mat depth(map->vpKeyFrames[i]->depthImage.size().y, map->vpKeyFrames[i]->depthImage.size().x, CV_16UC1, map->vpKeyFrames[i]->depthImage.data());
                const CameraModel& mcCamera = tracker->GetCamera();

                // TODO: make a list of pointclouds for each kf, only replace those have been changed in BA
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud = geometry::PointClouds::rgbdToPointCloud(
                            mcCamera, rgb, depth, map->vpKeyFrames[i]->se3CfromW.inverse(), map->vpKeyFrames[i]->rgbIsBgr_, step);
                *fullCloud_long += *thisCloud;
                nfullcloud ++;
            }
        else
            for (unsigned int i = nfullcloud*step; i < nCams; i += step) {
                cv::Mat rgb(map->vpKeyFrames[i]->rgbImage.size().y, map->vpKeyFrames[i]->rgbImage.size().x, CV_8UC3, map->vpKeyFrames[i]->rgbImage.data());
                cv::Mat depth(map->vpKeyFrames[i]->depthImage.size().y, map->vpKeyFrames[i]->depthImage.size().x, CV_16UC1, map->vpKeyFrames[i]->depthImage.data());
                const CameraModel& mcCamera = tracker->GetCamera();

                // TODO: make a list of pointclouds for each kf, only replace those have been changed in BA
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud = geometry::PointClouds::rgbdToPointCloud(
                            mcCamera, rgb, depth, map->vpKeyFrames[i]->se3CfromW.inverse(), map->vpKeyFrames[i]->rgbIsBgr_, step);
                *fullCloud += *thisCloud;
                nfullcloud ++;
            }

        sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2());
        if (map->didFullBA)
            pcl::toROSMsg(*fullCloud_long, *msg);
        else
            pcl::toROSMsg(*fullCloud, *msg);
        msg->header.frame_id = world_frame;//
        msg->header.stamp = ros::Time::now();

        vis_pointcloud_pub.publish(msg);
        lastCloudPublished_ = ros::Time::now();
    }

}

void MapVisualization::publishCrtPointCloud(const Map* map, const Tracker* tracker, const ros::Publisher &vis_pointcloud_pub, const std::string& world_frame,
                                                double vis_publish_interval_)
{
    static ros::Time lastCloudPublished_ = ros::Time(0);
    if (vis_pointcloud_pub.getNumSubscribers() > 0 &&
            (ros::Time::now() - lastCloudPublished_).toSec() >= vis_publish_interval_)
    {
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr crtCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // the current frame point cloud
        cv::Mat rgb(map->mCurrentkf->rgbImage.size().y, map->mCurrentkf->rgbImage.size().x, CV_8UC3, map->mCurrentkf->rgbImage.data());
        cv::Mat depth(map->mCurrentkf->depthImage.size().y, map->mCurrentkf->depthImage.size().x, CV_16UC1, map->mCurrentkf->depthImage.data());
        const CameraModel& mcCamera = tracker->GetCamera();

        // TODO: make a list of pointclouds for each kf, only replace those have been changed in BA
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud = geometry::PointClouds::rgbdToPointCloud(
                    mcCamera, rgb, depth, map->mCurrentkf->se3CfromW.inverse(), map->mCurrentkf->rgbIsBgr_);
        *crtCloud = *thisCloud;

        sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*crtCloud, *msg);
        msg->header.frame_id = world_frame;//
        msg->header.stamp = ros::Time::now();

        vis_pointcloud_pub.publish(msg);

        lastCloudPublished_ = ros::Time::now();
    }

}

void MapVisualization::publishMapVisualization(const Map* map, const Tracker* tracker, const ros::Publisher &camera_pub,
                                               const ros::Publisher &point_pub, const std::string& world_frame, const ros::Publisher &camera_pubsec,
                                               const bool dualcamera)
{
    bool useDifWorldFrame = false; // use a different world frame for rviz visualization

    static visualization_msgs::Marker camera_marker;
    static visualization_msgs::Marker camera_markersec[AddCamNumber];// second camera
    static visualization_msgs::Marker point_marker;
    static visualization_msgs::Marker campath_marker;//visualize camera path
    static visualization_msgs::Marker camkey_marker;

    static visualization_msgs::Marker cameranum_marker;
    static visualization_msgs::Marker cameranum_markersec;// second camera
    static visualization_msgs::Marker cameraconnect_marker[AddCamNumber];// second camera

    camera_marker.header.frame_id = world_frame;
    camera_marker.header.stamp = ros::Time::now();
    camera_marker.ns = "map";
    camera_marker.id = 0;
    camera_marker.action = visualization_msgs::Marker::ADD;
    camera_marker.pose.position.x = 0;
    camera_marker.pose.position.y = 0;
    camera_marker.pose.position.z = 0;
    camera_marker.pose.orientation.x = 0.0;
    camera_marker.pose.orientation.y = 0.0;
    camera_marker.pose.orientation.z = 0.0;
    camera_marker.pose.orientation.w = 1.0;
    camera_marker.scale.x = 0.01;
    camera_marker.scale.y = 0.01;
    camera_marker.scale.z = 0.01;
    camera_marker.color.r = 1.0f;//165.0/255.0f;
    camera_marker.color.g = 0;//30.0/255.0f;
    camera_marker.color.b = 0;//55.0/255.0f;
    camera_marker.color.a = 1.0f;
    camera_marker.lifetime = ros::Duration();
    camera_marker.type = visualization_msgs::Marker::LINE_LIST;

    if (dualcamera)
    for (int cn = 0; cn < AddCamNumber; cn ++){
        camera_markersec[cn].header.frame_id = world_frame;
        camera_markersec[cn].header.stamp = ros::Time::now();
        camera_markersec[cn].ns = "map";
        camera_markersec[cn].id = 4 + cn;
        camera_markersec[cn].action = visualization_msgs::Marker::ADD;
        camera_markersec[cn].pose.position.x = 0;
        camera_markersec[cn].pose.position.y = 0;
        camera_markersec[cn].pose.position.z = 0;
        camera_markersec[cn].pose.orientation.x = 0.0;
        camera_markersec[cn].pose.orientation.y = 0.0;
        camera_markersec[cn].pose.orientation.z = 0.0;
        camera_markersec[cn].pose.orientation.w = 1.0;
        camera_markersec[cn].scale.x = 0.01;
        camera_markersec[cn].scale.y = 0.01;
        camera_markersec[cn].scale.z = 0.01;
        camera_markersec[cn].color.r = 1.0f;//165.0/255.0f;
        camera_markersec[cn].color.g = 0;//30.0/255.0f;
        camera_markersec[cn].color.b = 0;//55.0/255.0f;
        camera_markersec[cn].color.a = 0.5f;
        camera_markersec[cn].lifetime = ros::Duration();
        camera_markersec[cn].type = visualization_msgs::Marker::LINE_LIST;

        cameraconnect_marker[cn].header.frame_id = world_frame;
        cameraconnect_marker[cn].header.stamp = ros::Time::now();
        cameraconnect_marker[cn].ns = "map";
        cameraconnect_marker[cn].id = 7;
        cameraconnect_marker[cn].action = visualization_msgs::Marker::ADD;
        cameraconnect_marker[cn].pose.position.x = 0;
        cameraconnect_marker[cn].pose.position.y = 0;
        cameraconnect_marker[cn].pose.position.z = 0;
        cameraconnect_marker[cn].pose.orientation.x = 0.0;
        cameraconnect_marker[cn].pose.orientation.y = 0.0;
        cameraconnect_marker[cn].pose.orientation.z = 0.0;
        cameraconnect_marker[cn].pose.orientation.w = 1.0;
        cameraconnect_marker[cn].scale.x = 0.01;
        cameraconnect_marker[cn].scale.y = 0.01;
        cameraconnect_marker[cn].scale.z = 0.01;
        cameraconnect_marker[cn].color.r = 1.0f;//165.0/255.0f;
        cameraconnect_marker[cn].color.g = 0;//30.0/255.0f;
        cameraconnect_marker[cn].color.b = 0;//55.0/255.0f;
        cameraconnect_marker[cn].color.a = 0.7f;
        cameraconnect_marker[cn].lifetime = ros::Duration();
        cameraconnect_marker[cn].type = visualization_msgs::Marker::LINE_LIST;
    }

    cameranum_marker.header.frame_id = world_frame;
    cameranum_marker.header.stamp = ros::Time::now();
    cameranum_marker.ns = "map";
    cameranum_marker.id = 5;
    cameranum_marker.action = visualization_msgs::Marker::ADD;
    cameranum_marker.pose.orientation.x = 0.0;
    cameranum_marker.pose.orientation.y = 0.0;
    cameranum_marker.pose.orientation.z = 0.0;
    cameranum_marker.pose.orientation.w = 1.0;
    cameranum_marker.color.r = 1.0;
    cameranum_marker.color.a = 1.0;
    cameranum_marker.scale.x = 0.15;
    cameranum_marker.scale.y = 0.15;
    cameranum_marker.scale.z = 0.15;
    cameranum_marker.lifetime = ros::Duration();
    cameranum_marker.text = "CAM1";
    cameranum_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    cameranum_markersec.header.frame_id = world_frame;
    cameranum_markersec.header.stamp = ros::Time::now();
    cameranum_markersec.ns = "map";
    cameranum_markersec.id = 6;
    cameranum_markersec.action = visualization_msgs::Marker::ADD;
    cameranum_markersec.pose.orientation.x = 0.0;
    cameranum_markersec.pose.orientation.y = 0.0;
    cameranum_markersec.pose.orientation.z = 0.0;
    cameranum_markersec.pose.orientation.w = 1.0;
    cameranum_markersec.color.r = 1.0;
    cameranum_markersec.color.a = 1.0;
    cameranum_markersec.scale.x = 0.15;
    cameranum_markersec.scale.y = 0.15;
    cameranum_markersec.scale.z = 0.15;
    cameranum_markersec.lifetime = ros::Duration();
    cameranum_markersec.text = "CAM2";
    cameranum_markersec.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    campath_marker.header.frame_id = world_frame;
    campath_marker.header.stamp = ros::Time::now();
    campath_marker.ns = "map";
    campath_marker.id = 1;
    campath_marker.action = visualization_msgs::Marker::ADD;
    campath_marker.pose.position.x = 0;
    campath_marker.pose.position.y = 0;
    campath_marker.pose.position.z = 0;
    campath_marker.pose.orientation.x = 0.0;
    campath_marker.pose.orientation.y = 0.0;
    campath_marker.pose.orientation.z = 0.0;
    campath_marker.pose.orientation.w = 1.0;
    campath_marker.scale.x = 0.005;
    campath_marker.scale.y = 0.01;
    campath_marker.scale.z = 0.01;
    campath_marker.color.r = 85.0/255.0f;
    campath_marker.color.g = 107.0/255.0f;
    campath_marker.color.b = 47.0/255.0f;
    campath_marker.color.a = 1.0f;
    campath_marker.lifetime = ros::Duration();
    campath_marker.type = visualization_msgs::Marker::LINE_STRIP;

    point_marker.header.frame_id = world_frame;
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = "map";
    point_marker.id = 2;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = 0;
    point_marker.pose.position.y = 0;
    point_marker.pose.position.z = 0;
    point_marker.pose.orientation.x = 0.0;
    point_marker.pose.orientation.y = 0.0;
    point_marker.pose.orientation.z = 0.0;
    point_marker.pose.orientation.w = 1.0;
    point_marker.color.r = 1.0f;
    point_marker.color.g = 0.0f;
    point_marker.color.b = 0.0f;
    point_marker.color.a = 1.0;
    point_marker.scale.x = 0.02;
    point_marker.scale.y = 0.02;
    point_marker.scale.z = 0.02;
    point_marker.lifetime = ros::Duration();
    point_marker.type = visualization_msgs::Marker::POINTS;

    camkey_marker.header.frame_id = world_frame;
    camkey_marker.header.stamp = ros::Time::now();
    camkey_marker.ns = "map";
    camkey_marker.id = 3;
    camkey_marker.action = visualization_msgs::Marker::ADD;
    camkey_marker.pose.position.x = 0;
    camkey_marker.pose.position.y = 0;
    camkey_marker.pose.position.z = 0;
    camkey_marker.pose.orientation.x = 0.0;
    camkey_marker.pose.orientation.y = 0.0;
    camkey_marker.pose.orientation.z = 0.0;
    camkey_marker.pose.orientation.w = 1.0;
    camkey_marker.scale.x = 0.015;
    camkey_marker.scale.y = 0.015;
    camkey_marker.scale.z = 0.01;
    camkey_marker.color.r = 1.0;
    camkey_marker.color.g = 0.0;
    camkey_marker.color.b = 1.0;
    camkey_marker.color.a = 1.0f;
    camkey_marker.lifetime = ros::Duration();
    camkey_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // read access: shared lock
    boost::shared_lock<boost::shared_mutex> lock(map->mutex);

    point_marker.points.resize(map->vpPoints.size());
    point_marker.colors.resize(map->vpPoints.size());

    for (unsigned int i = 0; i < map->vpPoints.size(); i++) {
        if (useDifWorldFrame){
            Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                  -1.0, 0, 0, // a world frame which pointing downward.
                                  0, -1.0, 0);
            TooN::Vector<3> mappointpose;
            mappointpose = datam * map->vpPoints[i]->v3WorldPos;

            point_marker.points[i].x = mappointpose[0];
            point_marker.points[i].y = mappointpose[1];
            point_marker.points[i].z = mappointpose[2];
        } else {
            point_marker.points[i].x = map->vpPoints[i]->v3WorldPos[0];
            point_marker.points[i].y = map->vpPoints[i]->v3WorldPos[1];
            point_marker.points[i].z = map->vpPoints[i]->v3WorldPos[2];
        }
        point_marker.colors[i].a = 1.0;

        boost::shared_ptr<KeyFrame> kf = map->vpPoints[i]->pPatchSourceKF.lock();
        int gray = kf->aLevels[map->vpPoints[i]->nSourceLevel].im[map->vpPoints[i]->irCenter];

        if (!map->vpPoints[i]->nSourceCamera)
        {
            point_marker.colors[i].b = 1.0;
            point_marker.colors[i].r = point_marker.colors[i].g = gray /255.0;
        }
        else{
            point_marker.colors[i].r = 1.0;
            point_marker.colors[i].g = point_marker.colors[i].b = gray /255.0;
        }
    }

    unsigned int nCams = map->vpKeyFrames.size();
    camera_marker.points.resize((nCams+1)*6);
    camera_marker.colors.resize((nCams+1)*6);
    camkey_marker.points.resize(nCams+1);
    unsigned int ii = 0;
    for (unsigned int i = 0; i < nCams+1; i++) {

        SE3<> camPose;
        if (i < nCams)
            camPose = map->vpKeyFrames[i]->se3CfromW.inverse();
        else
            camPose = tracker->GetCurrentPose().inverse();

        if (useDifWorldFrame){
            Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                         -1.0, 0, 0, // a world frame which pointing downward.
                                         0, -1.0, 0);
            SO3<> rot = SO3<>();
            rot = datam;
            camPose = rot * camPose;
        }
        Vector<3> T = camPose.get_translation();
        geometry_msgs::Point camPos;

        camPos = geometry_msgs_point(T);

        Vector<3> dX, dY, dZ;
        if (i < nCams){
            dX[0] = 0.075; dX[1] = 0.0;  dX[2] = 0.0;
            dY[0] = 0.0;   dY[1] = 0.05; dY[2] = 0.0;
            dZ[0] = 0.0;   dZ[1] = 0.0;  dZ[2] = 0.1;
        }
        else{
            dX[0] = 0.1; dX[1] = 0.0;  dX[2] = 0.0;
            dY[0] = 0.0;   dY[1] = 0.1; dY[2] = 0.0;
            dZ[0] = 0.0;   dZ[1] = 0.0;  dZ[2] = 0.1;
        }

        camera_marker.colors[ii].r = 255.0/255.0f;
        camera_marker.colors[ii].g = 0.0f;
        camera_marker.colors[ii].b = 0.0f;
        camera_marker.points[ii++] = camPos;

        Vector<3> to = camPose*dX;
        camera_marker.colors[ii].r = 255.0/255.0f;
        camera_marker.colors[ii].g = 0.0f;
        camera_marker.colors[ii].b = 0.0f;
        camera_marker.points[ii++] = geometry_msgs_point(to);

        camera_marker.colors[ii].r = 0.0f;
        camera_marker.colors[ii].g = 255.0/255.0f;;
        camera_marker.colors[ii].b = 0.0f;
        camera_marker.points[ii++] = camPos;
        to = camPose*dY;
        camera_marker.colors[ii].r = 0.0f;
        camera_marker.colors[ii].g = 255.0/255.0f;;
        camera_marker.colors[ii].b = 0.0f;
        camera_marker.points[ii++] = geometry_msgs_point(to);

        camera_marker.colors[ii].r = 0.0f;
        camera_marker.colors[ii].g = 0.0f;
        camera_marker.colors[ii].b = 255.0/255.0f;;
        camera_marker.points[ii++] = camPos;
        to = camPose*dZ;
        camera_marker.colors[ii].r = 0.0f;
        camera_marker.colors[ii].g = 0.0f;
        camera_marker.colors[ii].b = 255.0/255.0f;;
        camera_marker.points[ii++] = geometry_msgs_point(to);

        camkey_marker.points[i] = camPos;
    }

    /// camera poses for other cameras
    for (int cn = 0; cn < AddCamNumber; cn ++)
      if (dualcamera && map->vpKeyFramessec[cn].size()){
        unsigned int nCams0 = map->vpKeyFrames.size();
        unsigned int nCams = map->vpKeyFramessec[cn].size();
        camera_markersec[cn].points.resize((nCams+1)*6);
        camera_markersec[cn].colors.resize((nCams+1)*6);
        camkey_marker.points.resize(nCams0+nCams+1);

        cameraconnect_marker[cn].points.resize((nCams+1)*2);
        cameraconnect_marker[cn].colors.resize((nCams+1)*2);

        unsigned int ii = 0;
        for (unsigned int i = 0; i <nCams0; i ++){
            SE3<> camPose = map->vpKeyFrames[i]->se3CfromW.inverse();
            if (useDifWorldFrame){
                Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                             -1.0, 0, 0, // a world frame which pointing downward.
                                             0, -1.0, 0);
                SO3<> rot = SO3<>();
                rot = datam;
                camPose = rot * camPose;
            }
            Vector<3> T = camPose.get_translation();
            geometry_msgs::Point camPos;

            camPos = geometry_msgs_point(T);
            camkey_marker.points[i] = camPos;
        }
        for (unsigned int i = 0; i < nCams+1; i++) {
            SE3<> camPose;
            if (i < nCams)
                camPose = map->vpKeyFramessec[cn][i]->se3CfromW.inverse();
            else
                camPose = tracker->GetCurrentPosesec(cn).inverse();

            if (useDifWorldFrame){
                Matrix<3> datam = TooN::Data(0, 0, 1.0,//Rww1, because the roll and pitch angles are in
                                             -1.0, 0, 0, // a world frame which pointing downward.
                                             0, -1.0, 0);
                SO3<> rot = SO3<>();
                rot = datam;
                camPose = rot * camPose;
            }

            Vector<3> T = camPose.get_translation();
            geometry_msgs::Point camPos;

            camPos = geometry_msgs_point(T);

            Vector<3> dX, dY, dZ;
            if (i < nCams){
                dX[0] = 0.075; dX[1] = 0.0;  dX[2] = 0.0;
                dY[0] = 0.0;   dY[1] = 0.05; dY[2] = 0.0;
                dZ[0] = 0.0;   dZ[1] = 0.0;  dZ[2] = 0.1;
            }
            else{
                dX[0] = 0.1; dX[1] = 0.0;  dX[2] = 0.0;
                dY[0] = 0.0;   dY[1] = 0.1; dY[2] = 0.0;
                dZ[0] = 0.0;   dZ[1] = 0.0;  dZ[2] = 0.1;
            }

            camera_markersec[cn].colors[ii].r = 255.0/255.0f;
            camera_markersec[cn].colors[ii].g = 0.0f;
            camera_markersec[cn].colors[ii].b = 105.0/255.0f;
            camera_markersec[cn].colors[ii].a = 0.5f;
            camera_markersec[cn].points[ii++] = camPos;

            Vector<3> to = camPose*dX;
            camera_markersec[cn].colors[ii].r = 255.0/255.0f;
            camera_markersec[cn].colors[ii].g = 0.0f;
            camera_markersec[cn].colors[ii].b = 105.0/255.0f;
            camera_markersec[cn].colors[ii].a = 0.5f;
            camera_markersec[cn].points[ii++] = geometry_msgs_point(to);

            camera_markersec[cn].colors[ii].r = 0.0f;
            camera_markersec[cn].colors[ii].g = 255.0/255.0f;;
            camera_markersec[cn].colors[ii].b = 105.0/255.0f;
            camera_markersec[cn].colors[ii].a = 0.5f;
            camera_markersec[cn].points[ii++] = camPos;
            to = camPose*dY;
            camera_markersec[cn].colors[ii].r = 0.0f;
            camera_markersec[cn].colors[ii].g = 255.0/255.0f;;
            camera_markersec[cn].colors[ii].b = 105.0/255.0f;
            camera_markersec[cn].colors[ii].a = 0.5f;
            camera_markersec[cn].points[ii++] = geometry_msgs_point(to);

            camera_markersec[cn].colors[ii].r = 105.0/255.0f;
            camera_markersec[cn].colors[ii].g = 0.0f;
            camera_markersec[cn].colors[ii].b = 255.0/255.0f;;
            camera_markersec[cn].colors[ii].a = 0.5f;
            camera_markersec[cn].points[ii++] = camPos;
            to = camPose*dZ;
            camera_markersec[cn].colors[ii].r = 105.0/255.0f;
            camera_markersec[cn].colors[ii].g = 0.0f;
            camera_markersec[cn].colors[ii].b = 255.0/255.0f;;
            camera_markersec[cn].colors[ii].a = 0.5f;
            camera_markersec[cn].points[ii++] = geometry_msgs_point(to);

            camkey_marker.points[nCams0 + i] = camPos;

            if ((i < nCams)&&map->vpKeyFramessec[cn][i]->mAssociatedinFinalQueue){
                cameraconnect_marker[cn].colors[i*2].r = 0.5f;
                cameraconnect_marker[cn].colors[i*2].g = 0.0f;
                cameraconnect_marker[cn].colors[i*2].b = 0.0f;
                cameraconnect_marker[cn].points[i*2] = camPos;
                camPose = map->vpKeyFrames[map->vpKeyFramessec[cn][i]->nAssociatedKf]->se3CfromW.inverse();
                camPos = geometry_msgs_point(camPose.get_translation());
                cameraconnect_marker[cn].colors[i*2+1].r = 1.0f;
                cameraconnect_marker[cn].colors[i*2+1].g = 1.0f;
                cameraconnect_marker[cn].colors[i*2+1].b = 1.0f;
                cameraconnect_marker[cn].points[i*2+1] = camPos;
            }
        }

        // CHOSE a random keyframe to label its number
        int kfnum = (rand()%map->vpKeyFrames.size());
        SE3<> camPose = map->vpKeyFrames[kfnum]->se3CfromW.inverse();
        cameranum_marker.pose.position.x = camPose.get_translation()[0];
        cameranum_marker.pose.position.y = camPose.get_translation()[1];
        cameranum_marker.pose.position.z = camPose.get_translation()[2];

        if (map->vpKeyFrames[kfnum]->mAssociatedinFinalQueue){
            camPose = map->vpKeyFramessec[cn][map->vpKeyFrames[kfnum]->nAssociatedKf]->se3CfromW.inverse();
            cameranum_markersec.pose.position.x = camPose.get_translation()[0];
            cameranum_markersec.pose.position.y = camPose.get_translation()[1];
            cameranum_markersec.pose.position.z = camPose.get_translation()[2];
        }
        else{
            cameranum_markersec.pose.position.x = 0;
            cameranum_markersec.pose.position.y = 0;
            cameranum_markersec.pose.position.z = 0;
        }

    }

    lock.unlock();

    /// camera path, chose one camera for visualizaiton
    if (true)//(!dualcamera)
    {
        SE3<> camPose = tracker->GetCurrentPose().inverse();
        if (useDifWorldFrame){
            Matrix<3> datam = TooN::Data(0, 0, 1,//Rww1, because the roll and pitch angles are in
                                         -1, 0, 0, // a world frame which pointing downward.
                                         0, -1, 0);
            SO3<> rot = SO3<>();
            rot = datam;
            camPose = rot * camPose;
        }
        Vector<3> T = camPose.get_translation();
        geometry_msgs::Point camPos = geometry_msgs_point(T);
        campath_marker.points.push_back(camPos);
    }
    else{
        SE3<> camPose = tracker->GetCurrentPosesec(0).inverse();
        Vector<3> T = camPose.get_translation();
        geometry_msgs::Point camPos = geometry_msgs_point(T);
        campath_marker.points.push_back(camPos);
    }

    point_pub.publish(point_marker);
    camera_pub.publish(camera_marker);
    for (int cn = 0; cn < AddCamNumber; cn ++)
        if (dualcamera && map->vpKeyFramessec[cn].size())
        {
            camera_pubsec.publish(camera_markersec[cn]);
    //        camera_pubsec.publish(cameranum_markersec);
    //        camera_pub.pu+blish(cameranum_marker);
            camera_pubsec.publish(cameraconnect_marker[cn]);
        }

    camera_pub.publish(campath_marker);
    camera_pub.publish(camkey_marker);
}

void MapVisualization::renderDebugImage(cv::Mat& rgb_cv, const Tracker* tracker, const Map* map) {
    // draw interest points
    const ptam::KeyFrame& kf = tracker->GetCurrentKeyFrame();
    for (unsigned int i = 0; i < LEVELS; i++) {
        //        if ((i == 0))
        //            continue;
        const ptam::Level& lev = kf.aLevels[i];
        int scale = 1 << i;
        
        cv::Vec3b color;
        switch(i) {
        case 0: color = cv::Vec3b(255, 0, 0); break;
        case 1: color = cv::Vec3b(0, 255, 0); break;
        case 2: color = cv::Vec3b(255, 255, 0); break;
        case 3: color = cv::Vec3b(255, 0, 255); break;
        }

        for (unsigned int j = 0; j < lev.vMaxCorners.size(); j++) {
            int x = lev.vMaxCorners[j].x*(scale);
            int y = lev.vMaxCorners[j].y*(scale);
            rectangle(rgb_cv, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                      (cv::Scalar) color/*cv::Vec3b(0, 0, 0)*/, CV_FILLED);
        }
    }

    // draw map points
    for (unsigned int i = 0; i < map->vpPoints.size(); i++) {
        MapPoint &p= *(map->vpPoints[i]);

        if (p.nSourceCamera)// not from the master camera
            continue;

        p.TData.Project(p.v3WorldPos, kf.se3CfromW, camera.get());
        if (!p.TData.bInImage) {
            continue;
        }
        int x = p.TData.v2Image[0];
        int y = p.TData.v2Image[1];

        rectangle(rgb_cv, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                  (cv::Scalar) cv::Vec3b(0, 0, 255), CV_FILLED);
    }

    // draw successful matches
    for (std::map<boost::shared_ptr<MapPoint>, Measurement>::const_iterator mit = kf.mMeasurements.begin();
         mit != kf.mMeasurements.end(); mit++) {
        int x = mit->second.v2RootPos[0];
        int y = mit->second.v2RootPos[1];

        cv::circle(rgb_cv,cv::Point(x,y),4,cv::Scalar(0,255,255));
    }
}

void MapVisualization::renderDebugImageSec(cv::Mat& rgb_cv, const Tracker* tracker, const Map* map) {
    // draw interest points
    const ptam::KeyFrame& kf = tracker->GetCurrentsecKeyFrame();
    for (unsigned int i = 0; i < LEVELS; i++) {
        //        if ((i == 0))
        //            continue;
        const ptam::Level& lev = kf.aLevels[i];
        int scale = 1 << i;

        cv::Vec3b color;
        switch(i) {
        case 0: color = cv::Vec3b(255, 0, 0); break;
        case 1: color = cv::Vec3b(0, 255, 0); break;
        case 2: color = cv::Vec3b(255, 255, 0); break;
        case 3: color = cv::Vec3b(255, 0, 255); break;
        }

        for (unsigned int j = 0; j < lev.vMaxCorners.size(); j++) {
            int x = lev.vMaxCorners[j].x*(scale);
            int y = lev.vMaxCorners[j].y*(scale);
            rectangle(rgb_cv, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                      (cv::Scalar) color/*cv::Vec3b(0, 0, 0)*/, CV_FILLED);
        }
    }

    // draw map points
    for (unsigned int i = 0; i < map->vpPoints.size(); i++) {
        MapPoint &p= *(map->vpPoints[i]);

        if (!p.nSourceCamera)// from the master camera
            continue;

        p.TData.Project(p.v3WorldPos, kf.se3CfromW, camerasec[p.nSourceCamera - 1].get());
        if (!p.TData.bInImage) {
            continue;
        }
        int x = p.TData.v2Image[0];
        int y = p.TData.v2Image[1];

        rectangle(rgb_cv, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                  (cv::Scalar) cv::Vec3b(0, 0, 255), CV_FILLED);
    }

    // draw successful matches
    for (std::map<boost::shared_ptr<MapPoint>, Measurement>::const_iterator mit = kf.mMeasurements.begin();
         mit != kf.mMeasurements.end(); mit++) {
        int x = mit->second.v2RootPos[0];
        int y = mit->second.v2RootPos[1];

        cv::circle(rgb_cv,cv::Point(x,y),4,cv::Scalar(0,255,255));
    }
}

void MapVisualization::renderRefImage(cv::Mat& rgb_cv, const Tracker* tracker) {
    // draw interest points
    const ptam::KeyFrame& kf = tracker->GetRefFrame();
    for (unsigned int i = 0; i < LEVELS; i++) {
        //        if ((i == 0))
        //            continue;
        const ptam::Level& lev = kf.aLevels[i];
        int scale = 1 << i;

        cv::Vec3b color;
        switch(i) {
        case 0: color = cv::Vec3b(255, 0, 0); break;
        case 1: color = cv::Vec3b(0, 255, 0); break;
        case 2: color = cv::Vec3b(0, 0, 255); break;
        case 3: color = cv::Vec3b(255, 0, 255); break;
        }

        CVD::Image<CVD::Rgb<CVD::byte> > frameRef;
        CVD::Image<CVD::byte> frameRefbw;
        frameRefbw.resize(kf.aLevels[i].im.size());
        frameRef.resize(kf.aLevels[i].im.size());
        CVD::copy(kf.aLevels[i].im, frameRefbw);
        CVD::convert_image(frameRefbw,frameRef);
        cv::Mat img(frameRef.size().y, frameRef.size().x, CV_8UC3, frameRef.data());

        for (unsigned int j = 0; j < lev.vMaxCorners.size(); j++) {
            int x = lev.vMaxCorners[j].x*(scale);
            int y = lev.vMaxCorners[j].y*(scale);
            rectangle(rgb_cv, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                      (cv::Scalar) color/*cv::Vec3b(0, 0, 0)*/, CV_FILLED);
        }

        for (unsigned int j = 0; j < lev.vMaxCorners.size(); j++) {
            int x = lev.vMaxCorners[j].x;
            int y = lev.vMaxCorners[j].y;
            rectangle(img, cv::Point2f(x-1, y-1), cv::Point2f(x+1, y+1),
                      (cv::Scalar) color/*cv::Vec3b(0, 0, 0)*/, CV_FILLED);
        }

        switch(i) {
        case 0: cv::imshow("0", img); break;
        case 1: cv::imshow("1", img); break;
        case 2: cv::imshow("2", img); break;
        case 3: cv::imshow("3", img); break;
        }
    }

}

void MapVisualization::writeFrame(const sensor_msgs::ImageConstPtr& rgb_img_msg,
                                  const sensor_msgs::ImageConstPtr& depth_img_msg,
                                  int id) {
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_bridge::CvImagePtr cv_ptr_depth;
    try
    {
        cv_ptr_rgb = cv_bridge::toCvCopy(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_depth = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::MONO16);

        std::stringstream bgrname, depthname;
        bgrname << "frame" << id << ".png";
        depthname << "frame" << id << ".yml";
        cv::imwrite(bgrname.str(),cv_ptr_rgb->image);

        cv::FileStorage fs(depthname.str(), cv::FileStorage::WRITE);
        fs << "depth" << cv_ptr_depth->image;
        fs.release();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
