#ifndef POLYNOMIALCAMERA_H
#define POLYNOMIALCAMERA_H

#include <opencv2/opencv.hpp>
#include "CameraModel.h"
namespace ptam{
// Camera model based on the polynomial model employed by OpenV
class PolynomialCamera: public CameraModel {
public:
	PolynomialCamera(const char* file);

	// Recalculates the camera model parameters. Has to be overridden
	virtual void RefreshParams();
	
	// Projects from camera z=1 plane to pixel coordinates, with radial distortion
	virtual TooN::Vector<2> Project(const TooN::Vector<2>& camframe);
    // Projects from camera z=1 plane to pixel coordinates, without radial distortion
    virtual TooN::Vector<2> Project_ud(const TooN::Vector<2>& camframe);

	// Inverse operation
	virtual TooN::Vector<2> UnProject(const TooN::Vector<2>& imframe);
    // Inverse operation, without radial distortion
    virtual TooN::Vector<2> UnProject_ud(const TooN::Vector<2>& imframe);
    virtual TooN::Vector<2> UnProjectSafe(const TooN::Vector<2>& imframe) const ;

	// Projection jacobian
	virtual TooN::Matrix<2,2> GetProjectionDerivs();
	
	virtual bool Invalid() {return invalid;}
	virtual double LargestRadiusInImage() {return largestRadius * scalingFactor;}
	virtual double OnePixelDist() {return onePixelDist;}
	
private:
	cv::Mat_<double> projLookupX, projLookupY;
	cv::Mat_<double> projLookupU, projLookupV;
	cv::Mat_<double> cameraMatrix, distCoeffs;
	TooN::Vector<2> defaultSize;
	TooN::Vector<2> lastProjPos;
    TooN::Vector<2> lastProjPos_im;
	double onePixelDist;
	double largestRadius, largestSqRadius;
	bool invalid;
	double scalingFactor;
	
	// Creates one quarter of the lookup table
	void CreateLookupTableSection(double incX, double incY);
	// Inserts a single value into the lookup table
	bool InsertProjectionLookup(double x, double y);
	// Triggers the creation of the full lookup table
	void InitLookupTable();
};

} // namespace

#endif
