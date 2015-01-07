#ifndef CVDOPENCV_HELPER_H
#define CVDOPENCV_HELPER_H

#include <opencv2/core/core.hpp>
#include <TooN/TooN.h>

namespace cvdopencv_helper
{

// need to know the data type of mat, double, int...
template<int r, int c, class T>
inline TooN::Matrix<r, c> mat2toonmatrix(cv::Mat mat)//
{
    TooN::Matrix<r, c, T> matrix;
    for (int i = 0; i < r; i ++){
        for (int j = 0; j < c; j ++){
            matrix(i, j) = mat.at<T>(r, c);
        }
    }
    return matrix;
}

// type T should be CV_64FC1
template<int r, int c>
inline cv::Mat toonmatrix2mat(TooN::Matrix<r, c, double> M)//
{
    return cv::Mat(M.num_rows(),M.num_cols(),CV_64FC1,(M.my_data));
}

}

#endif // CVDOPENCV_HELPER_H
