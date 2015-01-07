/*
 * DepthViewer
 *
 *  Created on: Feb 16, 2011
 *      Author: scherer
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>
#include <sstream>
#include <string>

#include <gvars3/instances.h>

#include "ptam/CameraCalibrator.h"
#include "ptam/VideoBridgeROS.h"

using namespace std;
using namespace GVars3;

namespace ptam_ros {

class CalibrationNode {
public:
    CalibrationNode() : bridge_(VideoBridgeROS::getInstance())
    {
         sub_depth_ = nh_.subscribe("image", 1, &CalibrationNode::imageCallback, this);
    }

    ~CalibrationNode()
    {
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
        bridge_.pushROSImage(img_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_depth_;

    VideoBridgeROS& bridge_;

};

} // namespace rgbdslam


int main(int argc, char **argv)
{
    ros::init(argc, argv, "c");

    ptam_ros::CalibrationNode node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    GUI.LoadFile("calibrator_settings.cfg");

    GUI.StartParserThread();
    atexit(GUI.StopParserThread); // Clean up readline when program quits

    GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, SILENT);

    try
      {
        CameraCalibrator c;
        c.Run();
      }
    catch(CVD::Exceptions::All e)
      {
        std::cout << endl;
        cout << "!! Failed to run CameraCalibrator; got exception. " << endl;
        cout << "   Exception was: " << endl;
        cout << e.what << endl;
      }

    return 0;
}

