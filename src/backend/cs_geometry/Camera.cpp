#include "Camera.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

using namespace cs_geom;
using namespace cv;
using namespace std;

Camera::Camera()
{
    good_ = false;
}

Camera::Camera(const std::string& calib_file)
{
    cout << "Pose Graph camera calib file: " << calib_file << endl;
    this->readFromFile(calib_file);
}

struct SimpleMatrix
{
    int rows, cols;
    double* data;

    SimpleMatrix(int rows, int cols, double* data)
        : rows(rows), cols(cols), data(data)
    {}
};

void operator >> (const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
//    node["rows"] >> rows;
    rows = node["rows"].as<int>();// >> rows;
    assert(rows == m.rows);
    cols = node["cols"].as<int>();// >> cols;
    assert(cols == m.cols);
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
        m.data[i] = data[i].as<double>();
}

void Camera::readFromFile(const std::string& calib_file)
{
    static const char WIDTH_YML_NAME[]  = "image_width";
    static const char HEIGHT_YML_NAME[] = "image_height";
    static const char K_YML_NAME[]      = "camera_matrix";
    static const char D_YML_NAME[]      = "distortion_coefficients";

    // First: Use Yaml to extract width_, height_, K_, D_
//    std::ifstream in(calib_file.c_str());
    try {
        // belows are using the old yaml api
//        YAML::Parser parser(in);
//        if (!parser) {
//            cerr << "Unable to create YAML parser for camera calibration" << endl;
//            good_ = false;
//            return;
//        }

//        YAML::Node doc;
//        parser.GetNextDocument(doc);
        YAML::Node doc = YAML::LoadFile(calib_file);// using the new api

        width_ = doc[WIDTH_YML_NAME].as<int>();//  >> width_;
        height_ = doc[HEIGHT_YML_NAME].as<int>();

        // Read camera matrix:
        K_ = cv::Mat(3, 3, CV_64FC1, 0.0);
        SimpleMatrix K(3, 3, (double*) K_.data);
        doc[K_YML_NAME] >> K;

        // Read distortion coefficients:
        const YAML::Node& D_node = doc[D_YML_NAME];
        int D_rows, D_cols;
        D_rows = D_node["rows"].as<int>();
        D_cols = D_node["cols"].as<int>();
        D_ = cv::Mat(1, 8, CV_64FC1, 0.0);
        SimpleMatrix D(1, D_rows*D_cols, (double*) D_.data);
        doc[D_YML_NAME] >> D;
    }
    catch (YAML::Exception& e) {
        cerr << "Exception parsing YAML camera calibration:" << endl << e.what() << endl;
        good_ = false;
        return;
    }

    // Second: Extract values from K_, D_
    // K contains only four relevant parameters, assuming pinhole camera model with square pixels:
    fx_ = K_.at<double>(0,0);
    fy_ = K_.at<double>(1,1);
    cx_ = K_.at<double>(0,2);
    cy_ = K_.at<double>(1,2);

    assert(D_.size[0] == 1);
    int lenD = D_.size[1];
    assert(lenD >= 4);
    assert(lenD <= 8);

    // D can contain up to 8 distortion coefficients:
    k1_ = D_.at<double>(0,0);
    k2_ = D_.at<double>(0,1);
    p1_ = D_.at<double>(0,2);
    p2_ = D_.at<double>(0,3);
    k3_ = lenD > 4? D_.at<double>(0,4) : 0.0f;
    k4_ = lenD > 5? D_.at<double>(0,5) : 0.0f;
    k5_ = lenD > 6? D_.at<double>(0,6) : 0.0f;
    k6_ = lenD > 7? D_.at<double>(0,7) : 0.0f;

    cout << "Camera: building unprojection LUT..." << endl;
    unprojectMap_ = cv::Mat(height_,width_,CV_32FC2);
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            cv::Vec2d p(x,y);
            cv::Mat pt(1,1,CV_64FC2);
            pt.at<Vec2d>(0) = p;
            cv::undistortPoints(pt, pt, K_, D_);
            unprojectMap_.at<Vec2f>(y,x) = Vec2f(pt.at<Vec2d>(0)[0],pt.at<Vec2d>(0)[1]);
        }
    }
    cout << "... done bulding unprojection LUT" << endl;
    good_ = true;
}

Camera::~Camera()
{

}

cv::Point3d Camera::unprojectPixelTo3D(const cv::Point2i& p) const
{
    cv::Point2d xy = unprojectPixel(p);
    return cv::Point3d(xy.x, xy.y, 1.0);
}


cv::Point3d Camera::unprojectPixelTo3D(const cv::Point2d& p) const
{
    cv::Point2d xy = unprojectPixel(p);
    return cv::Point3d(xy.x, xy.y, 1.0);
}

Eigen::Vector3d Camera::unprojectPixelTo3D(const Eigen::Vector2d& p) const
{
    cv::Point2d xy = unprojectPixel(cv::Point2d(p[0],p[1]));
    return Eigen::Vector3d(xy.x, xy.y, 1.0);
}

cv::Point2d Camera::project3DtoPixel(const cv::Point3d& p) const
{
    // see http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Point2d xy1(p.x/p.z, p.y/p.z);
    cv::Point2d xy2 = distortPixel(xy1);

    // apply intrinsic matrix
    cv::Point2d uv(fx_*xy2.x + cx_, fy_*xy2.y + cy_);
    return uv;
}

Eigen::Vector2d Camera::project3DtoPixel(const Eigen::Vector3d& p) const
{
    // see http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Point2d xy1(p[0]/p[2], p[1]/p[2]);
    cv::Point2d xy2 = distortPixel(xy1);

    // apply intrinsic matrix
    Eigen::Vector2d uv(fx_*xy2.x + cx_, fy_*xy2.y + cy_);
    return uv;
}

bool Camera::isInImage(const cv::Point2d& uv) const
{
    return uv.x >= 0 && uv.x < width_ &&
            uv.y >= 0 && uv.y < height_;
}

bool Camera::isInImage(const Eigen::Vector2d& uv) const
{
    return uv[0] >= 0 && uv[0] < width_ &&
            uv[1] >= 0 && uv[1] < height_;
}

bool Camera::isVisible(const cv::Point3d& p) const
{
    if (p.z <= 0.)
        return false;

    cv::Point2d uv = project3DtoPixel(p);
    return isInImage(uv);
}

bool Camera::isVisible(const Eigen::Vector3d& p) const
{
    if (p[2] <= 0.)
        return false;

    Eigen::Vector2d uv = project3DtoPixel(p);
    return isInImage(uv);
}

cv::Point2d Camera::distortPixel(const cv::Point2d& xy1) const
{
    // see http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    const double& x1 = xy1.x;
    const double& y1 = xy1.y;
    double r2 = x1*x1 + y1*y1;
    double q = (1 + r2*(k1_ + r2*(k2_ + r2*k3_))) /
            (1 + r2*(k4_ + r2*(k5_ + r2*k6_)));
    double x1y1 = x1*y1;
    cv::Point2d xy2(x1*q + 2*p1_*x1y1 + p2_*(r2 + 2*x1*x1),
                    y1*q + p1_*(r2 + 2*y1*y1) + 2*p2_*x1y1);
    return xy2;
}

cv::Point2d Camera::unprojectPixel(const cv::Point2i& p) const
{
    cv::Vec2f res = unprojectMap_.at<cv::Vec2f>(p.y,p.x);
    return cv::Point2d(res[0],res[1]);
}

cv::Point2d Camera::unprojectPixel(const cv::Point2d& p) const
{
    // undistort using unprojectMap_ and bilinear interpolation
    int x0 = (int) p.x; int x1 = x0 + 1;
    int y0 = (int) p.y; int y1 = y0 + 1;

    assert(x0 >= 0);        assert(y0 >= 0);
    assert(x1 < width_);    assert(y1 < height_);

    const cv::Vec2f& p0 = unprojectMap_.at<cv::Vec2f>(y0,x0);
    const cv::Vec2f& p1 = unprojectMap_.at<cv::Vec2f>(y0,x1);
    const cv::Vec2f& p2 = unprojectMap_.at<cv::Vec2f>(y1,x0);
    const cv::Vec2f& p3 = unprojectMap_.at<cv::Vec2f>(y1,x1);

    double dx = p.x - (double) x0;
    double dy = p.y - (double) y0;
    double dx1 = 1.0 - dx;
    double dy1 = 1.0 - dy;

    cv::Vec2f res = (p0*dx1 + p1*dx)*dy1 + (p2*dx1 + p3*dx)*dy;
    return cv::Point2d(res[0],res[1]);
}

Eigen::Vector2d Camera::unprojectPixel(const Eigen::Vector2d& p) const
{
    // undistort using unprojectMap_ and bilinear interpolation
    int x0 = (int) p[0]; int x1 = x0 + 1;
    int y0 = (int) p[1]; int y1 = y0 + 1;

    assert(x0 >= 0);        assert(y0 >= 0);
    assert(x1 < width_);    assert(y1 < height_);

    const cv::Vec2f& p0 = unprojectMap_.at<cv::Vec2f>(y0,x0);
    const cv::Vec2f& p1 = unprojectMap_.at<cv::Vec2f>(y0,x1);
    const cv::Vec2f& p2 = unprojectMap_.at<cv::Vec2f>(y1,x0);
    const cv::Vec2f& p3 = unprojectMap_.at<cv::Vec2f>(y1,x1);

    double dx = p[0] - (double) x0;
    double dy = p[1] - (double) y0;
    double dx1 = 1.0 - dx;
    double dy1 = 1.0 - dy;

    cv::Vec2f res = (p0*dx1 + p1*dx)*dy1 + (p2*dx1 + p3*dx)*dy;
    return Eigen::Vector2d(res[0],res[1]);
}

