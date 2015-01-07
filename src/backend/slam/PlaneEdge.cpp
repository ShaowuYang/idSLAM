#include "PlaneEdge.h"

using namespace cslam;

PlaneEdge::PlaneEdge() :
    BaseUnaryEdge<2, g2o::SE3Quat, g2o::VertexSE3>()
{

}


bool PlaneEdge::read(std::istream& is)
{
    assert(false);
}

bool PlaneEdge::write(std::ostream& os) const
{
    assert(false);
}


void PlaneEdge::linearizeOplus() {
    //Xi - estimate the jacobian numerically
    VertexXiType* vi = static_cast<VertexXiType*>(_vertices[0]);

    if (vi->fixed())
        return;

#ifdef G2O_OPENMP
    vi->lockQuadraticForm();
#endif

    const double delta = 1e-9;
    const double scalar = 1.0 / (2*delta);
    ErrorVector error1;
    ErrorVector errorBeforeNumeric = _error;

    double add_vi[VertexXiType::Dimension];
    std::fill(add_vi, add_vi + VertexXiType::Dimension, 0.0);
    // add small step along the unit vector in each dimension
    for (int d = 0; d < VertexXiType::Dimension; ++d) {
        vi->push();
        add_vi[d] = delta;
        vi->oplus(add_vi);
        computeError();
        error1 = _error;
        vi->pop();
        vi->push();
        add_vi[d] = -delta;
        vi->oplus(add_vi);
        computeError();
        vi->pop();
        add_vi[d] = 0.0;

        _jacobianOplusXi.col(d) = scalar * (error1 - _error);
    } // end dimension

//    std::cout << "jacobian: " << std::endl;
//    std::cout << _jacobianOplusXi << std::endl;

    _error = errorBeforeNumeric;
#ifdef G2O_OPENMP
    vi->unlockQuadraticForm();
#endif
}
