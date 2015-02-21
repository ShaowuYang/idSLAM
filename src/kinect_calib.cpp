 /*
 * PTAMNode.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: yang, scherer
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <highgui.h>

#include <boost/scoped_ptr.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/signals.hpp>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/image_convert.h>
#include <TooN/wls.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "ptam/Tracker.h"

#include "BaseSLAMNode.h"
#include "MapVisualization.h"

#include "conversions/conversions.h"

using namespace CVD;
using namespace std;

class Cameras_calib{
public:
    Cameras_calib() : nh_private_("~")
    {

        if (!nh_private_.getParam("kinect_file", cam_para_path))
            cam_para_path = "data/allpose.txt";

        pos_log_.open("data/kinect1kinect2.txt");
        cout <<"kinects logfile open?: " << pos_log_.is_open() << endl;
        if (pos_log_.is_open()){
            pos_log_.setf(std::ios::fixed, std::ios::floatfield);
            pos_log_.precision(10);
        }
        else
            return;

        Load_cam_para();

    }

    ~Cameras_calib()
    {
        if (pos_log_.is_open())
            pos_log_.close();
    }

    SE3<> Load_cam_para()
    {
        // camera1: front, camera2: right
        // free kinect: t, board: b, robot: r
        SE3<> Tt1, Ttf, Tfb, Tt2, Ttr, Trb;

        ifstream cam_para_table;
        string table_path = cam_para_path;
        double temp1;
        cam_para_table.open(table_path.c_str());
        assert(cam_para_table.is_open());

        Matrix<3> cam;
        // cam1
        // free cam SLAM pose
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Tt1.get_translation()[i] = temp1*1000;
        }
        Tt1.get_rotation() = cam; // Tcw
        cout << Tt1.get_translation() << endl;
        cout << Tt1.get_rotation().get_matrix() <<endl;
        // free cam toolbox front pad pose
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Ttf.get_translation()[i] = temp1;
        }
        Ttf.get_rotation() = cam;// Tcb
        cout << Ttf.get_translation() << endl;
        cout << Ttf.get_rotation().get_matrix() << endl;
        // front cam toolbox pose
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Tfb.get_translation()[i] = temp1;
        }
        Tfb.get_rotation() = cam;// Tcb
        cout << Tfb.get_translation() << endl;
        cout << Tfb.get_rotation().get_matrix() << endl;
        // cam2
        // free cam SLAM pose
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Tt2.get_translation()[i] = temp1*1000;
        }
        Tt2.get_rotation() = cam;// Tcw
        cout << Tt2.get_translation() << endl;
        cout << Tt2.get_rotation().get_matrix() << endl;
        // free cam toolbox right pad pose
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Ttr.get_translation()[i] = temp1;
        }
        Ttr.get_rotation() = cam;// Tcb
        cout << Ttr.get_translation() << endl;
        cout << Ttr.get_rotation().get_matrix() << endl;
        // right cam toolbox pose
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Trb.get_translation()[i] = temp1;
        }
        Trb.get_rotation() = cam;// Tcb
        cout << Trb.get_translation() << endl;
        cout << Trb.get_rotation().get_matrix() << endl;
        // calibrate kinect1fromkinect2
        // pad1 from pad2
        SE3<> Tb1b2;// Tb1w * Twb2
        Tb1b2 = (Ttf.inverse() * Tt1) * (Tt2.inverse() * Ttr);
        SE3<> Tk1k2;// Tk1b1 * Tb1b2 * Tb2k2
        Tk1k2 = Tfb * Tb1b2 * Trb.inverse();
        // save paramters
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j++)
                pos_log_ << Tk1k2.get_rotation().get_matrix()(i, j) << " ";
            pos_log_ << Tk1k2.get_translation()[i] << endl;
        }
    }


    string cam_para_path;

    ros::NodeHandle n;
    ros::NodeHandle nh_private_;

    ofstream pos_log_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameras_calib");

    Cameras_calib node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

//    ros::waitForShutdown();

    return 0;
}

