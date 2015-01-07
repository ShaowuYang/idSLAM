/*
 * RGBDLogger.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: scherer
 */

#include <image_geometry/pinhole_camera_model.h>
#include <cv.h>
#include <highgui.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace cv;
using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
    if (argc != 2 && argc != 14) {
        std::cout << "usage: rgbd_to_pyl framename [r11 r12 r13 t1 r21 r22 r23 r2 r31 r32 r33 t3]" << std::endl;
    }

    bool have_transform = argc == 14;

    stringstream ssbgr, ssdepth, ssply;
    ssbgr << argv[1] << ".png";
    ssdepth << argv[1] << ".yml";
    ssply << argv[1] << ".ply";

    // read images
    Mat bgr = imread(ssbgr.str().c_str());
    FileStorage fs(ssdepth.str().c_str(), FileStorage::READ);
    Mat depth;
    fs["depth"] >> depth;
    fs.release();

    if (!(depth.type() == CV_16U || depth.type() == CV_16S)) {
        std::cerr << "invalid type of depth image: " << depth.type() << std::endl;
        exit(EXIT_FAILURE);
    }

    // count number of valid depth values == number of points in cloud
    int npoints = 0;
    for (int y = 0; y < bgr.rows; y++) {
        for (int x = 0; x < bgr.cols; x++) {
            double d = depth.at<uint16_t>(y,x)/1000.0;
            if (d > 0 && !isnan(d)) {
                npoints++;
            }
        }
    }

    FILE* out = fopen(ssply.str().c_str(),"w");
    fprintf(out,"ply\n");
    fprintf(out,"format ascii 1.0\n");
    fprintf(out,"element vertex %i\n",npoints);
    fprintf(out,"property float x\n");
    fprintf(out,"property float y\n");
    fprintf(out,"property float z\n");
    fprintf(out,"property uchar red\n");
    fprintf(out,"property uchar green\n");
    fprintf(out,"property uchar blue\n");
    fprintf(out,"end_header\n");


    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    if (have_transform) {
        pose(0,0) = atof(argv[2]);  pose(0,1) = atof(argv[3]);  pose(0,2) = atof(argv[4]);  pose(0,3) = atof(argv[5]);
        pose(1,0) = atof(argv[6]);  pose(1,1) = atof(argv[7]);  pose(1,2) = atof(argv[8]);  pose(1,3) = atof(argv[9]);
        pose(2,0) = atof(argv[10]); pose(2,1) = atof(argv[11]); pose(2,2) = atof(argv[12]); pose(2,3) = atof(argv[13]);
        pose = pose.inverse();
    }
    float uv[2];
    float uv_undist[2];
    cv::Mat K = (Mat_<float>(3, 3) << 520.962, 0, 310.143, 0, 520.039, 240.679, 0, 0, 1);
    cv::Mat D = (Mat_<float>(1, 4) << 0,0,0,0);

    for (int y = 0; y < bgr.rows; y++) {
        uv[1] = y;
        for (int x = 0; x < bgr.cols; x++) {
            uv[0] = x;
            double d = depth.at<uint16_t>(y,x)/1000.0;
            if (d > 0 && !isnan(d)) {
                cv::Mat src_pt(1, 1, CV_32FC2, uv);
                cv::Mat dst_pt(1, 1, CV_32FC2, uv_undist);
                cv::undistortPoints(src_pt,dst_pt, K, D);

                Vector3d p = Vector3d(uv_undist[0], uv_undist[1], 1)*d;

                // transform to world:
                p = pose*p;
                unsigned char b = bgr.at<Vec3b>(y,x)[0];
                unsigned char g = bgr.at<Vec3b>(y,x)[1];
                unsigned char r = bgr.at<Vec3b>(y,x)[2];

                fprintf(out,"%f %f %f %u %u %u\n",p(0), p(1), p(2), r, g, b);
            }
        }
    }

    fclose(out);

    return 0;
}
