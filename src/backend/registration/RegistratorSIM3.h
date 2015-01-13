#ifndef _CSLAM_REG_SIM3_H_
#define _CSLAM_REG_SIM3_H_

#include <TooN/wls.h>
#include <TooN/sim3.h>
#include <sophus/sim3.hpp>
#include "Registrator3P.h"

namespace backend {

class RegistratorSIM3 {
public:
    RegistratorSIM3(bool useSIM3 = true) : useSIM3_(useSIM3) {}

    void solve(const std::vector<Observation>& obsAB,
               const std::vector<Observation>& obsBA);

    Sophus::Sim3d aTb_sim3();
    Sophus::Sim3d bTa_sim3();

    Sophus::SE3d aTb_se3();
    Sophus::SE3d bTa_se3();
protected:
    void calcErrAB(const Observation& obsAB,
                   TooN::Vector<2>& err);
    void calcErrJacobianAB(const Observation& obsAB,
                           TooN::Vector<2>& err,
                           TooN::Matrix<2,7>& jac);

    void calcErrBA(const Observation& obsBA,
                   TooN::Vector<2>& err);
    void calcErrJacobianBA(const Observation& obsBA,
                           TooN::Vector<2>& err,
                           TooN::Matrix<2,7>& jac);

    bool useSIM3_;

    TooN::WLS<7> wls_;
    TooN::SIM3<> estATB_, estBTA_;
};

}

#endif /* _CSLAM_REG_SIM3_H_ */
