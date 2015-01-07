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

        if (!nh_private_.getParam("cam_imu_file", cam_para_path))
            cam_para_path = "data/cameras_wrt.txt";

        pos_log_.open("cam1cam2.txt");
        cout <<"cam1cam2 logfile open?: " << pos_log_.is_open() << endl;
        if (pos_log_.is_open()){
            pos_log_.setf(std::ios::fixed, std::ios::floatfield);
            pos_log_.precision(10);
        }

        Load_cam_para();

    }

    ~Cameras_calib()
    {
        if (pos_log_.is_open())
            pos_log_.close();
    }

    SE3<> Load_cam_para()
    {
        // camera1: downward, camera2: forward
        // trackingsystem: t, board: b, robot: r
        SE3<> Tdb, Tfb, Ttb, Ttr1, Ttr2;

        ifstream cam_para_table;
        string table_path = cam_para_path;
        double temp1, temp2, temp3, temp4;
        cam_para_table.open(table_path.c_str());
        assert(cam_para_table.is_open());

        Matrix<3> cam;
        // cam1
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Tdb.get_translation()[i] = temp1;
        }
        Tdb.get_rotation() = cam;
        // cam2
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Tfb.get_translation()[i] = temp1;
        }
        Tfb.get_rotation() = cam;

        // board in the tracking system frames
        for (int i = 0 ; i < 3; i ++){
            cam_para_table>>temp1;
            Ttb.get_translation()[i] = temp1;
        }
        cam_para_table>>temp1>>temp2>>temp3>>temp4;
        btTransform trans1(btQuaternion(temp1,
                                       temp2,
                                       temp3,
                                       temp4),
                          btVector3(Ttb.get_translation()[0],
                                    Ttb.get_translation()[1],
                                    Ttb.get_translation()[2]));
        btMatrix3x3 R1(trans1.getRotation());
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam(i, j) = R1[i][j];
            }
        }
        Ttb.get_rotation() = cam;
        cout << Ttb.get_rotation().get_matrix()[0] << "\n"
             << Ttb.get_rotation().get_matrix()[1] << "\n"
             << Ttb.get_rotation().get_matrix()[2] << "\n"
             << Ttb.get_translation() << "\n";
        // now we set the board as ground plane
//        Ttb = SE3<>();
        // robot1 in the tracking system frames
        for (int i = 0 ; i < 3; i ++){
            cam_para_table>>temp1;
            Ttr1.get_translation()[i] = temp1;
        }
        cam_para_table>>temp1>>temp2>>temp3>>temp4;
        btTransform trans2(btQuaternion(temp1,
                                       temp2,
                                       temp3,
                                       temp4),
                          btVector3(Ttr1.get_translation()[0],
                                    Ttr1.get_translation()[1],
                                    Ttr1.get_translation()[2]));
        btMatrix3x3 R2(trans2.getRotation());
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam(i, j) = R2[i][j];
            }
        }
        Ttr1.get_rotation() = cam;
        // robot2 in the tracking system frames
        for (int i = 0 ; i < 3; i ++){
            cam_para_table>>temp1;
            Ttr2.get_translation()[i] = temp1;
        }
        cam_para_table>>temp1>>temp2>>temp3>>temp4;
        btTransform trans3(btQuaternion(temp1,
                                       temp2,
                                       temp3,
                                       temp4),
                          btVector3(Ttr2.get_translation()[0],
                                    Ttr2.get_translation()[1],
                                    Ttr2.get_translation()[2]));
        btMatrix3x3 R3(trans3.getRotation());
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam(i, j) = R3[i][j];
            }
        }
        Ttr2.get_rotation() = cam;

        // b1fromb2, b1 is for the tracking markers,
        // b2 is the chess board frames
        SE3<> Tb1b2;
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j ++){
                cam_para_table>>temp1;
                cam(i, j) = temp1;
            }
            cam_para_table>>temp1;
            Tb1b2.get_translation()[i] = temp1;
        }
        Tb1b2.get_rotation() = cam;
        // calibrate cam1fromcam2
        SE3<> Tdf;
        Tdf = Tdb*Ttb.inverse()*Tb1b2.inverse()*Ttr1*Ttr2.inverse()*Ttb*Tb1b2*Tfb.inverse();
        // save paramters
        for (int i = 0; i < 3; i ++){
            for (int j = 0; j < 3; j++)
                pos_log_ << Tdf.get_rotation().get_matrix()(i, j) << " ";
            pos_log_ << Tdf.get_translation()[i] << endl;
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

