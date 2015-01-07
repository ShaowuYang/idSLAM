#ifndef CAMERAMODEL_H
#define CAMERAMODEL_H

#include <TooN/TooN.h>
#include <cvd/vector_image_ref.h>
#include <string>
#include <memory>

#define AddCamNumber 2 // additional camera number, besides the main camera

namespace ptam{
// Abstract base class for all camera models
class CameraModel {
public:
	CameraModel();

	void SetImageSize(TooN::Vector<2> v2ImageSize) {
		if(mvImageSize[0] != v2ImageSize[0] || mvImageSize[1] != v2ImageSize[1]) {
			mvImageSize = v2ImageSize;
			RefreshParams();
		}
	}
	void SetImageSize(CVD::ImageRef irImageSize) {SetImageSize(vec(irImageSize));};
	TooN::Vector<2> GetImageSize() const {return mvImageSize;};
	
	// Recalculates the camera model parameters. Has to be overridden
	virtual void RefreshParams() = 0;
	
	// Projects from camera z=1 plane to pixel coordinates, with radial distortion
	virtual TooN::Vector<2> Project(const TooN::Vector<2>& camframe) = 0;
	TooN::Vector<2> Project(CVD::ImageRef ir) { return Project(vec(ir)); }
    // Projects from camera z=1 plane to pixel coordinates, without radial distortion
    virtual TooN::Vector<2> Project_ud(const TooN::Vector<2>& camframe) = 0;
    TooN::Vector<2> Project_ud(CVD::ImageRef ir) { return Project_ud(vec(ir)); }

	// Inverse operation
    virtual TooN::Vector<2> UnProject(const TooN::Vector<2>& imframe) = 0;
    TooN::Vector<2> UnProject(CVD::ImageRef ir)  { return UnProject(vec(ir)); }
    virtual TooN::Vector<2> UnProjectSafe(const TooN::Vector<2>& imframe) const  = 0;// without modifing anything in the model
    // Inverse operation
    virtual TooN::Vector<2> UnProject_ud(const TooN::Vector<2>& imframe) = 0;
    TooN::Vector<2> UnProject_ud(CVD::ImageRef ir)  { return UnProject_ud(vec(ir)); }

	// Projection jacobian
	virtual TooN::Matrix<2,2> GetProjectionDerivs() = 0;
	
	virtual bool Invalid() =0;
	virtual double LargestRadiusInImage() = 0;
	virtual double OnePixelDist() = 0;
	
	// Creates a camera that matches the current configuration
    static CameraModel* CreateCamera(int camnum = 0);
	
private:
    static bool firstCreate, secondCreate;
	static std::auto_ptr<CameraModel> cameraPrototype;
    static std::auto_ptr<CameraModel> cameraPrototypesec;
	static bool polynomial;
    static bool polynomialsec;
	TooN::Vector<2> mvImageSize;
};
} // namespace

#endif
