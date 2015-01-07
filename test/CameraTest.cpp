#include <gtest/gtest.h>

#include <gvars3/instances.h>

#include <ptam/CameraModel.h>
#include <ptam/ATANCamera.h>
#include <ptam/PolynomialCamera.h>

// Factory functions creating different cameras
template <class T>
CameraModel* CreateCamera();

template <>
CameraModel* CreateCamera<ATANCamera>()
{


    GVars3::GUI.LoadFile("data/kinect-atan.cfg");
    return new ATANCamera("Camera");
}

template <>
CameraModel* CreateCamera<PolynomialCamera>()
{
    return new PolynomialCamera("data/kinect-poly.xml");
}

template <class T>
class CameraTest : public testing::Test {
protected:
    CameraTest(): camera_(CreateCamera<T>()) {}
    virtual ~CameraTest() { delete camera_; }

    CameraModel* const camera_;
};

// Run test case CameraTestGeneric on the following implementations:
using testing::Types;
typedef Types<ATANCamera, PolynomialCamera> CameraImplementations;
TYPED_TEST_CASE(CameraTest, CameraImplementations);

// Declare another test
TYPED_TEST(CameraTest, reprojection)
{
    const double tolerance = 1e-3;
    TooN::Vector<2> imageSize = this->camera_->GetImageSize();
    const int nSamples = 3;
    for (int i = 0; i < nSamples; i++) {
        TooN::Vector<2> impos, unproj, reproj;
        impos[0] = imageSize[0]*rand()/(RAND_MAX+1.0);
        impos[1] = imageSize[1]*rand()/(RAND_MAX+1.0);

        unproj = this->camera_->UnProject(impos);
        reproj = this->camera_->Project(unproj);

        EXPECT_NEAR (impos[0], reproj[0], tolerance);
        EXPECT_NEAR (impos[1], reproj[1], tolerance);
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
