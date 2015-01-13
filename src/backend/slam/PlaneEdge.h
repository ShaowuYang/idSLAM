#ifndef _CSLAM_PLANE_EDGE_H_
#define _CSLAM_PLANE_EDGE_H_

#include <sophus/se3.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>

namespace backend {

class PlaneEdge : public g2o::BaseUnaryEdge<2, g2o::SE3Quat, g2o::VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PlaneEdge();

    void computeError()
    {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

        Eigen::Matrix<double,4,4> est_wTc = v1->estimate().to_homogenious_matrix();
        Eigen::Matrix<double,4,4> meas_gTc = _measurement.to_homogenious_matrix();

        Eigen::Vector4d estPlane = est_wTc.row(2);
        Eigen::Vector4d measPlane = meas_gTc.row(2);
        Eigen::Vector3d cross = estPlane.head<3>().cross(measPlane.head<3>());
        _error[0] = cross.norm();
        _error[1] = est_wTc(2,3) - meas_gTc(2,3);

//        std::cout << "id: " << v1->id() << std::endl;
//        std::cout << "estPlane: " << estPlane.transpose() << std::endl;
//        std::cout << "measPlane: " << measPlane.transpose() << std::endl;
//        std::cout << "error: " << _error.transpose() << std::endl;
    }

    virtual void setMeasurement(const g2o::SE3Quat& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* d) {
        g2o::Vector7d v;
        v.setZero();
        for (int i = 0; i < 6; i++)
            v[i] = d[i];

        _measurement.fromVector(v);
        _inverseMeasurement = _measurement.inverse();

        return true;
    }

    virtual bool getMeasurementData(double* d) const {
        g2o::Vector7d v = _measurement.toVector();
        for (int i = 0; i < 7; i++)
            d[i] = v[i];
        return true;
    }

    int measurementDimension() const { return 7; }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 0.;}
//    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to);

        virtual void linearizeOplus();
};
}

#endif /* _CSLAM_PLANE_EDGE_H_ */
