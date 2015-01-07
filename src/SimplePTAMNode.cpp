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

#include "ptam/System.h"
#include "ptam/VideoBridgeROS.h"

using namespace std;
using namespace GVars3;

namespace ptam_ros {

class PTAMNode {
public:
    PTAMNode() : bridge_(VideoBridgeROS::getInstance())
    {
        sub_depth_ = nh_.subscribe("image", 1, &PTAMNode::imageCallback, this);
    }

    ~PTAMNode()
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

    ptam_ros::PTAMNode node;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    GUI.LoadFile("settings.cfg");

    GUI.StartParserThread(); // Start parsing of the console input
    atexit(GUI.StopParserThread);

    try
    {
        System s;
        s.Run();
    }
    catch(CVD::Exceptions::All e)
    {
        cout << endl;
        cout << "!! Failed to run system; got exception. " << endl;
        cout << "   Exception was: " << endl;
        cout << e.what << endl;
    }

    return 0;
}

