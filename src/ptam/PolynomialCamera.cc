#include <iostream>
#include <cassert>
#include <TooN/helpers.h>
#include <cvd/vector_image_ref.h>
#include "PolynomialCamera.h"

using namespace TooN;
using namespace cv;
using namespace CVD;
using namespace std;
using namespace ptam;

PolynomialCamera::PolynomialCamera(const char* file):
	onePixelDist(0), largestRadius(0), largestSqRadius(0), invalid(true), scalingFactor(1) {
	
    cout << "Read camera parameters..." << file <<"\n" << flush;
	// Read camera parameters
	FileStorage fs(file, CV_STORAGE_READ);
    if(!fs.isOpened()) {
		cerr << "Unable to read calibration file: " << file << endl;
		exit(1);
	}

	fs["M1"] >> cameraMatrix;
	fs["D1"] >> distCoeffs;
	Mat_<int> matSize(2,1);
	fs["size"] >> matSize;
	defaultSize[0] = matSize(0);
	defaultSize[1] = matSize(1);
	fs.release();
	
	// Create lookup table
	InitLookupTable();
	
	// By default we chose the image size used for calibration
	SetImageSize(defaultSize);
}

void PolynomialCamera::InitLookupTable() {
	// Create a lookup table for inverse projection. The table has 
	// twice the size of the image resultion, such that we can
	// do projections at 0.5 pixel resolution. We also keep the largest
	// radius.
	cout << "Creating projection lookup table... " << flush;
	
	projLookupX = Mat_<double>(defaultSize[1]*2, defaultSize[0]*2, -1.0);
	projLookupY = Mat_<double>(defaultSize[1]*2, defaultSize[0]*2, -1.0);
	projLookupU = Mat_<double>(defaultSize[1]*2, defaultSize[0]*2, -1e10),
	projLookupV = Mat_<double>(defaultSize[1]*2, defaultSize[0]*2, -1e10);
	
	// This is very slow and inefficient but MATLAB can't find the inverse to
	// the projection functions. However, we only have to do this once.
	const double step = 5e-4;
	
	// Lookups for bottom right
	CreateLookupTableSection(step, step);
	// Top right
	CreateLookupTableSection(step, -step);
	// Bottom left
	CreateLookupTableSection(-step, step);
	// Top left
	CreateLookupTableSection(-step, -step);
	
	largestRadius = sqrt(largestSqRadius);
	
	cout << "done" << endl;
	
	// Check if all elements of the lookup tables are initialized
	for(int v = 0; v<projLookupX.rows; v++)
		for(int u = 0; u<projLookupX.cols; u++) {
			if(projLookupU(v, u) < -1e9) {
				cerr << "Uninitialized lookup table entry: " << u << "; " << v << endl;
				exit(1);
			}
		}
	projLookupU = Mat_<double>(); // No longer needed
	projLookupV = Mat_<double>(); // No longer needed
}

void PolynomialCamera::RefreshParams() {
	// Just calculate a new scaling factor in case the image size changed
	scalingFactor = GetImageSize()[0] / defaultSize[0];
	
	// Code adapted from ATAN camera model:
	// work out world radius of one pixel
	// (This only really makes sense for square-ish pixels)
	TooN::Vector<2> v2Center = UnProject(GetImageSize() / 2);
	TooN::Vector<2> v2RootTwoAway = UnProject(GetImageSize() / 2 + vec(ImageRef(1,1)));
	TooN::Vector<2> v2Diff = v2Center - v2RootTwoAway;
	onePixelDist = sqrt(v2Diff * v2Diff) / sqrt(2.0);
//    cout << "scalingFactor: " << scalingFactor << "onePixelDist" << onePixelDist
//         <<"v2Diff" << v2Center<< ", " << v2RootTwoAway<<endl;
}

void PolynomialCamera::CreateLookupTableSection(double incX, double incY) {
	// Depending on the sign of incX and incY, we either create the
	// upper-left, bottom-left, upper-right or bottom-right quarter of
	// the lookup table.

	// We require the maximum row length for those cases, in which some
	// pixel are outside the vertical lookup-table bounds.
	unsigned int xMax = 0;

	for(unsigned int y=0; ; y++) {
		bool inserted = true;
		unsigned int insertCount = 0;
		
		for(unsigned int x=0; inserted || x<=xMax; x++) {
			inserted = InsertProjectionLookup(x*incX, y*incY);
			if(inserted) {
				insertCount++;
				if(x > xMax)
					xMax = x;
			}
		}
		
		// We stop inserting values if we find one y-coordinate, for which
		// all projected points exceed the vertical bounds of the lookup table.
		if(insertCount == 0)
			break;
	}
}

bool PolynomialCamera::InsertProjectionLookup(double x, double y) {
	// First perform projection of given coordinates
	TooN::Vector<2> camPos;
	camPos[0] = x;
	camPos[1] = y;
	TooN::Vector<2> imgPos = Project(camPos);
	
	// Find lookup table indices
	int lookupU = int(2*imgPos[0]+0.5);
	int lookupV = int(2*imgPos[1]+0.5);
	
	if(lookupU < 0 || lookupV < 0 || lookupU >= projLookupX.cols || lookupV >= projLookupX.rows)
		return false; // Outside of lookup table bounds
		
	// In case we already inserted a point, we calculate the distance of the
	// previous image point to the optimal lookup table entry. If nothing
	// was inserted before, this distance will be large.
	double du1 = lookupU/2.0 - projLookupU(lookupV, lookupU);
	double dv1 = lookupV/2.0 - projLookupV(lookupV, lookupU);
	
	// Calculate the same distance for the new point.
	double du2 = lookupU/2.0 - imgPos[0];
	double dv2 = lookupV/2.0 - imgPos[1];
	
	if(du2*du2 + dv2*dv2 < du1*du1 + dv1*dv1) {
		// This is a better entry for the lookup table
		projLookupU(lookupV, lookupU) = imgPos[0];
		projLookupV(lookupV, lookupU) = imgPos[1];
		projLookupX(lookupV, lookupU) = camPos[0];
		projLookupY(lookupV, lookupU) = camPos[1];
		
		// Update the maximum radius if neccessary	
		double r2 = camPos[0] * camPos[0] + camPos[1] * camPos[1];
		if(r2 > largestSqRadius)
			largestSqRadius = r2;		
	}
	return true;
}

TooN::Vector<2> PolynomialCamera::Project(const TooN::Vector<2>& camframe) {
	// Inserts camframe into the projection equations.
	// See http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

	lastProjPos = camframe;

	const double &f_x = cameraMatrix(0, 0), 
		&f_y = cameraMatrix(1,1),
		&c_x = cameraMatrix(0, 2), 
		&c_y = cameraMatrix(1, 2),
        &x_1 = camframe[0], &y_1 = camframe[1], // xn
		&k_1 = distCoeffs(0,0), &k_2 = distCoeffs(0,1), &k_3 = distCoeffs(0,4),
		&p_1 = distCoeffs(0,2), &p_2 = distCoeffs(0,3);

	const double r2 = x_1*x_1 + y_1*y_1; //r^2
	const double distFactor = 1 + k_1*r2 + k_2*r2*r2 + k_3*r2*r2*r2;
	
    const double xp = x_1 * distFactor + 2*p_1*x_1*y_1 + p_2*(r2 + 2*x_1*x_1); // xd
	const double yp = y_1 * distFactor + 2*p_2*x_1*y_1 + p_1*(r2 + 2*y_1*y_1);
	
	TooN::Vector<2> ret;
    ret[0] = scalingFactor * (f_x * xp + c_x); // xp
	ret[1] = scalingFactor * (f_y * yp + c_y);
	
	invalid = ret[0] < 0 || ret[0] >= GetImageSize()[0] ||
		ret[1] < 0 || ret[1] >= GetImageSize()[1];
	return ret;
}
TooN::Vector<2> PolynomialCamera::Project_ud(const TooN::Vector<2>& camframe) {
    // Inserts camframe into the projection equations.
    // See http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

    lastProjPos = camframe;

    const double &f_x = cameraMatrix(0, 0),
        &f_y = cameraMatrix(1,1),
        &c_x = cameraMatrix(0, 2),
        &c_y = cameraMatrix(1, 2),
        &x_1 = camframe[0], &y_1 = camframe[1];

    TooN::Vector<2> ret;
    ret[0] = scalingFactor * (f_x * x_1 + c_x); // xp
    ret[1] = scalingFactor * (f_y * y_1 + c_y);

    invalid = ret[0] < 0 || ret[0] >= GetImageSize()[0] ||
        ret[1] < 0 || ret[1] >= GetImageSize()[1];
    return ret;
}

TooN::Vector<2> PolynomialCamera::UnProject(const TooN::Vector<2>& imframe) {
	// Find best lookup positions
	float lookupU = 2*imframe[0] / scalingFactor;
	float lookupV = 2*imframe[1] / scalingFactor;
	int lookupUMinus = max(0, int(lookupU));
	int lookupVMinus = max(0, int(lookupV));
	int lookupUPlus = min(int(lookupU)+1, projLookupX.cols-1);
	int lookupVPlus = min(int(lookupV)+1, projLookupX.rows-1);
	
	// Perform bilinear interpolation
	float cu = lookupU - int(lookupU);
	float cv = lookupV - int(lookupV);

	float ixu1 = (1.0F - cu) * projLookupX(lookupVMinus, lookupUMinus) + cu * projLookupX(lookupVMinus, lookupUPlus);
	float ixu2 = (1.0F - cu) * projLookupX(lookupVPlus, lookupUMinus) + cu * projLookupX(lookupVPlus, lookupUPlus);
	lastProjPos[0] = (1.0F - cv)*ixu1 + cv*ixu2;
	
	float iyu1 = (1.0F - cu) * projLookupY(lookupVMinus, lookupUMinus) + cu * projLookupY(lookupVMinus, lookupUPlus);
	float iyu2 = (1.0F - cu) * projLookupY(lookupVPlus, lookupUMinus) + cu * projLookupY(lookupVPlus, lookupUPlus);
	lastProjPos[1] = (1.0F - cv)*iyu1 + cv*iyu2;
	
	invalid = false;
	return lastProjPos;
}

TooN::Vector<2> PolynomialCamera::UnProject_ud(const TooN::Vector<2>& imframe) {
    const double &f_x = cameraMatrix(0, 0),
        &f_y = cameraMatrix(1,1),
        &c_x = cameraMatrix(0, 2),
        &c_y = cameraMatrix(1, 2);

    lastProjPos[0] = (imframe[0]/scalingFactor - c_x) / f_x;
    lastProjPos[1] = (imframe[1]/scalingFactor - c_y) / f_y;

    invalid = false;
    return lastProjPos;
}

TooN::Vector<2> PolynomialCamera::UnProjectSafe(const TooN::Vector<2>& imframe) const {
    TooN::Vector<2> projpos;
    // Find best lookup positions
    float lookupU = 2*imframe[0] / scalingFactor;
    float lookupV = 2*imframe[1] / scalingFactor;
    int lookupUMinus = max(0, int(lookupU));
    int lookupVMinus = max(0, int(lookupV));
    int lookupUPlus = min(int(lookupU)+1, projLookupX.cols-1);
    int lookupVPlus = min(int(lookupV)+1, projLookupX.rows-1);

    // Perform bilinear interpolation
    float cu = lookupU - int(lookupU);
    float cv = lookupV - int(lookupV);

    float ixu1 = (1.0F - cu) * projLookupX(lookupVMinus, lookupUMinus) + cu * projLookupX(lookupVMinus, lookupUPlus);
    float ixu2 = (1.0F - cu) * projLookupX(lookupVPlus, lookupUMinus) + cu * projLookupX(lookupVPlus, lookupUPlus);
    projpos[0] = (1.0F - cv)*ixu1 + cv*ixu2;

    float iyu1 = (1.0F - cu) * projLookupY(lookupVMinus, lookupUMinus) + cu * projLookupY(lookupVMinus, lookupUPlus);
    float iyu2 = (1.0F - cu) * projLookupY(lookupVPlus, lookupUMinus) + cu * projLookupY(lookupVPlus, lookupUPlus);
    projpos[1] = (1.0F - cv)*iyu1 + cv*iyu2;

    return projpos;
}

TooN::Matrix<2,2> PolynomialCamera::GetProjectionDerivs() {
	// get the derivative of image frame wrt camera z=1 frame at the last computed projection
	// in the form (d im1/d cam1, d im1/d cam2)
	//             (d im2/d cam1, d im2/d cam2)

	const double &f_x = cameraMatrix(0,0), 
		&f_y = cameraMatrix(1,1),
		&x_1 = lastProjPos[0], &y_1 = lastProjPos[1],
		&k_1 = distCoeffs(0,0), &k_2 = distCoeffs(0,1), &k_3 = distCoeffs(0,4),
		&p_1 = distCoeffs(0,2), &p_2 = distCoeffs(0,3);

	TooN::Matrix<2, 2> ret;
	const double r2 = x_1*x_1 + y_1*y_1;
	
	const double term1 = k_2*r2*r2 + k_3*r2*r2*r2 + 2*p_1*y_1 + k_1*r2 + 1;
	const double term2 = 2*p_2*y_1;
	const double term3 = 2*k_1*x_1 + 4*k_2*x_1*r2 + 6*k_3*x_1*r2*r2;
	const double term4 = 2*k_1*y_1 + 4*k_2*y_1*r2 + 6*k_3*y_1*r2*r2;
	
	ret[0][0] = scalingFactor * f_x * (term1 + 6*p_2*x_1 + x_1*term3); // du/dx
	ret[1][0] = scalingFactor * f_y * (6*p_1*x_1 + term2 + y_1*term3); // dv/dx
	ret[0][1] = scalingFactor * f_x * (2*p_1*x_1 + term2 + x_1*term4); // du/dy
	ret[1][1] = scalingFactor * f_y * (term1 + 2*p_2*x_1 + y_1*term4); // dv/dy
	
	return ret;
}
