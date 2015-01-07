/*
  using code from opencv fundam.cpp, so that the findhomography function does not
  do LM optimization, and the maximal iteration could be controlled, in order to
  control time cost of findhomography.
  */
#ifndef HOMOGRAPHYSOLVER_H
#define HOMOGRAPHYSOLVER_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace ptam{
class HomographyEstimator
{
public:
    HomographyEstimator( int modelPoints, CvSize _modelSize = cvSize(3,3), int _maxBasicSolutions = 1 );

    int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model );
    bool refine( const CvMat* m1, const CvMat* m2,
                         CvMat* model, int maxIters );
    int runRANSAC( const CvMat* m1, const CvMat* m2, CvMat* model,
                                        CvMat* mask0, double reprojThreshold,
                                        double confidence, int maxIters );
protected:
    void computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error );
    int findInliers( const CvMat* m1, const CvMat* m2,
                             const CvMat* model, CvMat* error,
                             CvMat* mask, double threshold );
    bool getSubset( const CvMat* m1, const CvMat* m2,
                            CvMat* ms1, CvMat* ms2, int maxAttempts=1000 );
    bool checkSubset( const CvMat* m, int count );

    CvRNG rng;
    int modelPoints;
    CvSize modelSize;
    int maxBasicSolutions;
    bool checkPartialSubsets;
};

namespace homographyRANSACsolver
{
    cv::Mat findHomographyRANSAC(int &ransacinliers, cv::InputArray _points1, cv::InputArray _points2, int mininliers = 6, const int maxIters = 1000,
                                  double ransacReprojThreshold=3, cv::OutputArray _mask = cv::noArray() );
    int FindHomography_core( const CvMat* objectPoints, const CvMat* imagePoints, CvMat* __H, int mininlier,
                             const int maxIters_ = 1000, double ransacReprojThreshold=3, CvMat *mask CV_DEFAULT(0) );

};
}
#endif // HOMOGRAPHYSOLVER_H
