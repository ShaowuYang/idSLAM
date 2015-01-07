#ifndef _CS_GEOMETRY_CAMERA_H_
#define _CS_GEOMETRY_CAMERA_H_

#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace cs_geom {

/**
* Camera class. Contains camera calibration and performs projection / unprojection operations.
*/
class Camera
{
public:
    /*!
    * \brief Default constructor will not do anything. Objects created with this will return isGood as false.
    */
    Camera();

    /*!
    * \brief Constructs camera object from calibration file.
    * \param filename Filename that contains calibration data (OpenCV yaml format).
    */
    Camera(const std::string& filename);
    ~Camera();

    /*!
    * \brief Initializes existing camera object from calibration file.
    * \param filename Filename that contains calibration data (OpenCV yaml format).
    */
    void readFromFile(const std::string& filename);

    /*!
    * \brief Indicates whether creation of this object succeeded.
    * \return true for success, false otherwise.
    */
    bool isGood() { return good_; }

    /*!
    * \brief Project 3D coordinates to their corresponding pixel coordinates.
    * \param p 3D coordinates to be projected.
    * \return Pixel coordinates (as doubles).
    */
    cv::Point2d project3DtoPixel(const cv::Point3d& p) const;

    /*!
    * \brief Project 3D coordinates to their corresponding pixel coordinates.
    * \param p 3D coordinates to be projected.
    * \return Pixel coordinates (as doubles).
    */
    Eigen::Vector2d project3DtoPixel(const Eigen::Vector3d& p) const;

    /*!
    * \brief Checks whether provided pixel coordinates are within the image bounds.
    * \param p 2D pixel coordinates.
    * \return true if p is within the image, false otherwise.
    */
    bool isInImage(const cv::Point2d& p) const;

    /*!
    * \brief Checks whether provided pixel coordinates are within the image bounds.
    * \param p 2D pixel coordinates.
    * \return true if p is within the image, false otherwise.
    */
    bool isInImage(const Eigen::Vector2d& p) const;

    /*!
    * \brief Checks whether a provided 3D point is visible.
    * \param p 3D position.
    * \return true if the projection of p is within the image, false otherwise.
    */
    bool isVisible(const cv::Point3d& p) const;

    /*!
    * \brief Checks whether a provided 3D point is visible.
    * \param p 3D position.
    * \return true if the projection of p is within the image, false otherwise.
    */
    bool isVisible(const Eigen::Vector3d & p) const;

    /*!
    * \brief Apply camera's distortion model to "normalized" (i.e. divided by depth) image coordinates.
    * \param p Normalized image coorinates.
    * \return Distorted normalized image coordinates
    */
    cv::Point2d distortPixel(const cv::Point2d& p) const;

    /*!
    * \brief Unproject pixel coordinates to normalized (divided by depth) image coordinates.
    *        Since the input is pixel-aligned, this requires looking up one pixel in the unprojection LUT.
    * \param p Pixel Coordinates
    * \return Unprojected normalized image coordinates.
    */
    cv::Point2d unprojectPixel(const cv::Point2i& p) const;

    /*!
    * \brief Unproject pixel coordinates to their corresponding 3D ray.
    *        Since the input is pixel-aligned, this requires looking up one pixel in the unprojection LUT.
    * \param p 2D pixel coordinates to be unprojected.
    * \return Direction of the corresponding 3D ray with z == 1.
    */
    cv::Point3d unprojectPixelTo3D(const cv::Point2i& p) const;

    /*!
    * \brief Unproject pixel coordinates to normalized (divided by depth) image coordinates.
    *        Since the input is NOT pixel-aligned, this requires looking up 4 pixels in
    *        the unprojection LUT and computing a bilinear interpolation
    * \param p Pixel Coordinates
    * \return Unprojected normalized image coordinates.
    */
    cv::Point2d unprojectPixel(const cv::Point2d& p) const;
    Eigen::Vector2d unprojectPixel(const Eigen::Vector2d& p) const;

    /*!
    * \brief Unproject pixel coordinates to their corresponding 3D ray.
    *        Since the input is NOT pixel-aligned, this requires looking up 4 pixels in
    *        the unprojection LUT and computing a bilinear interpolation
    * \param p 2D pixel coordinates to be unprojected.
    * \return Direction of the corresponding 3D ray with z == 1.
    */
    cv::Point3d unprojectPixelTo3D(const cv::Point2d& p) const;
    Eigen::Vector3d unprojectPixelTo3D(const Eigen::Vector2d& p) const;

    cv::Mat_<double> K() const { return K_; }
    cv::Mat_<double> D() const { return D_; }

    inline int height() const { return height_; }
    inline int width() const { return width_; }

    inline double fx() const { return fx_; }
    inline double fy() const { return fy_; }
    inline double cx() const { return cx_; }
    inline double cy() const { return cy_; }

    inline double k1() const { return k1_; }
    inline double k2() const { return k2_; }
    inline double k3() const { return k3_; }
    inline double k4() const { return k4_; }
    inline double k5() const { return k5_; }
    inline double k6() const { return k6_; }
    inline double p1() const { return p1_; }
    inline double p2() const { return p2_; }

private:
    bool good_;  /// Did creation of this object succeed?

    int width_, height_;
    double fx_, fy_, cx_, cy_; /// Focal length, principal point (contents of K).
    double k1_, k2_, p1_, p2_, k3_, k4_, k5_, k6_; /// Distortion coefficients (radial and tangential).
    cv::Mat_<double> K_; /// Camera matrix as used by OpenCV
    cv::Mat_<double> D_; /// Distortion coefficients as used by OpenCV

    /// Unprojection lookup table:
    /// For each pixel position, we store its unprojected normalized image coordinates.
    /// We need such a LUT as the distortion function cannot be inverted analytically.
    cv::Mat_<cv::Vec2d> unprojectMap_;
};

}

#endif /* _CS_GEOMETRY_CAMERA_H_ */
