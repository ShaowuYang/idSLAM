/*
 * FivePointSolver.cpp
 *
 *  Created on: Apr 22, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#include "FivePointSolver.h"

#include <iostream>
#include <iomanip>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <opencv2/calib3d/calib3d.hpp>

#include "Math.h"
#include "TrivariatePolynomials.h"

using namespace Eigen;
using namespace std;

namespace cs_geom {

void FivePointHypothesis::get2RT1(const std::vector<cv::Point2d>& points1,
                                const std::vector<cv::Point2d>& points2,
                                Eigen::Matrix3d& R,
                                Eigen::Vector3d& T)
{
    // Implementation as described in Hartley, Zisserman, p. 258

    // need to use JacobiSVD as we are dealing with a degenerate case (E has 2
    // identical nonzero singular values, the third one is zero)
    JacobiSVD<Matrix3d> svd(E, Eigen::ComputeFullV | Eigen::ComputeFullU);

    Matrix3d U = svd.matrixU();
    Vector3d t = U.col(2);
    Matrix3d V = svd.matrixV();

    double detU = U.determinant();
    double detV = V.determinant();
    if ((detU > 0 && detV < 0) || (detU < 0 && detV > 0))
        V = -V;
    Matrix3d D;
    D << 0, 1, 0,    -1, 0, 0,   0, 0, 1;
    Matrix3d Ra = U*D*V.transpose();
    Matrix3d Rb = U*D.transpose()*V.transpose();

    Matrix3d RCand[4];
    RCand[0] = Ra;
    RCand[1] = Ra;
    RCand[2] = Rb;
    RCand[3] = Rb;

    Vector3d TCand[4];
    TCand[0] = t;
    TCand[1] = -t;
    TCand[2] = t;
    TCand[3] = -t;

    // check cheirality constraint for all 4 possible solutions
    int cheirality[4] = {0,0,0,0};
    for (int i = 0; i < 4; i++) {
        std::vector<Vector3d> X; // 3D point in view 1
        FivePointSolver::triangulate(points1,points2,RCand[i],TCand[i],X);

        for (int j = 0; j < (int) X.size(); j++) {
            // transform X into view 2
            Vector3d X2 = RCand[i]*X[j] + TCand[i];
            if (X2.z() > 0 && X[j].z() > 0) {
                // point visible in both views
                cheirality[i]++;
            }
        }
    }

    // find solution that satisfies the cheirality constraint best
    int iBest = 0;
    int nBest = 0;
    for (int i = 0; i < 4; i++) {
        if (cheirality[i] > nBest) {
            iBest = i;
            nBest = cheirality[i];
        }
    }

    R = RCand[iBest];
    T = TCand[iBest];
}

inline double square(double s) { return s*s; }

double FivePointHypothesis::reprojError2(const cv::Point2d& q1, const cv::Point2d& q2)
{
    // q1^T * E * q2
    double err = Eigen::Vector3d(q1.x, q1.y, 1.0).transpose()
            *E* Eigen::Vector3d(q2.x, q2.y, 1.0);
    return err*err;
}

double FivePointHypothesis::pointLineError2(const cv::Point2d& q1, const cv::Point2d& q2)
{
    // Zhang PAMI98: On the Optimization Criteria Used in Two-View Motion Analysis
    // q1^T*E
    Eigen::Vector3d l1 = E.col(0)*q1.x;
    l1 += E.col(1)*q1.y;
    l1 += E.col(2);

    double d1sq = square(l1[0]*q2.x + l1[1]*q2.y + l1[2]) /
            (l1[0]*l1[0] + l1[1]*l1[1]);

    // E*q2
    Eigen::Vector3d l2 = E.row(0)*q2.x;
    l2 += E.row(1)*q2.y;
    l2 += E.row(2);

    double d2sq = square(l2[0]*q1.x +l2[1]*q1.y + l2[2]) /
            (l2[0]*l2[0] + l2[1]*l2[1]);

    return d1sq + d2sq;
}

double FivePointHypothesis::sampsonError2(const cv::Point2d& q1, const cv::Point2d& q2)
{
    // sampson reprojection error (see Hartley, Zisserman p. 287)

    // compute E*x_2
    double Ex2x = q2.x*E(0,0) + q2.y*E(0,1) + E(0,2);
    double Ex2y = q2.x*E(1,0) + q2.y*E(1,1) + E(1,2);
    double Ex2z = q2.x*E(2,0) + q2.y*E(2,1) + E(2,2);

    // compute E^T *x1
    double ETx1x = q1.x*E(0,0) + q1.y*E(1,0) + E(2,0);
    double ETx1y = q1.x*E(0,1) + q1.y*E(1,1) + E(2,1);

    // compute (E*x_2) dot x_1
    double Ex2x1 = Ex2x*q1.x + Ex2y*q1.y + Ex2z;

    double error =  Ex2x1*Ex2x1 / ( Ex2x*Ex2x + Ex2y*Ex2y + ETx1x*ETx1x + ETx1y*ETx1y );

    return error;
}

FivePointHypothesis FivePointSolver::findEPreemptiveRANSAC(const std::vector<cv::Point2d>& points1,
                                                           const std::vector<cv::Point2d>& points2,
                                                           int nMaxO,
                                                           int nInitialH,
                                                           int blockSize)
{
    assert(points1.size() == points2.size());

    // determine the order in which observations will be evaluated
    std::vector<int> oInd;
    randPerm(points1.size(),oInd);

    // generate nInitialH hypotheses
    std::vector<FivePointHypothesis> h;
    while ((int) h.size() < nInitialH) {
        // sample five points
        std::vector<int> fiveInd;
        randSampleNoReplacement(points1.size(),5,fiveInd);

        // prepare points
        std::vector<cv::Point2d> q1;
        std::vector<cv::Point2d> q2;
        for (int i = 0; i < 5; i++) {
            q1.push_back(points1[fiveInd[i]]);
            q2.push_back(points2[fiveInd[i]]);
        }

        // compute associated essential matrices using the five point algorithm
        std::vector<Matrix3d> Efp;
        fivePointNisterGroebner(q1,q2,Efp);

        // store all of them for scoring
        for (int i = 0; i < (int) Efp.size() && (int) h.size() < nInitialH; i++) {
            h.push_back(FivePointHypothesis(fiveInd,Efp[i]));
        }
    }

    std::cout << "FivePointSolver::findEPreemptiveRANSAC: created " << nInitialH << "hypotheses." << std::endl;

    // start preemptive scoring
    int i = 0;
    int pr = preemption(i,nInitialH,blockSize);
    while (i < nMaxO && i < (int) points1.size() && pr > 1) {
        // observation oInd(i) consists of one pair of points

        // update score for all hypotheses w.r.t. observation oInd(i)
        for (int j = 0; j < (int) h.size(); j++) {
            h[j].score += h[j].scorePair(points1[oInd[i]],
                                         points2[oInd[i]]);
        }

        i++;
        int prnext = preemption(i,nInitialH,blockSize);
        if (prnext != pr) {
            // select best hypotheses
            std::nth_element(h.begin(),h.begin()+prnext,h.end(),FivePointHypothesis::compare);
            // now the first prnext elements of h contain the best hypotheses, erase the rest
            h.erase(h.begin()+prnext,h.end());
        }
        pr = prnext;
    }
    // preemptive scoring is done

    // select the single best hypothesis of possibly more than one remaining
    std::nth_element(h.begin(),h.begin()+1,h.end(),FivePointHypothesis::compare);

    std::cout << "preemptive scoring using " << i << " observations done." <<  std::endl;
    return h[0];
}

void computeConstraints(const std::vector<cv::Point2d>& q1, const std::vector<cv::Point2d>& q2, TriPoly3* F, Matrix3d& X, Matrix3d& Y, Matrix3d& Z, Matrix3d& W)
{
    const int nPoints = q1.size();

    // build matrix Q
    MatrixXd Q(nPoints,9);
    for (int i = 0; i < nPoints; i++) {
        Q.row(i) << q1[i].x*q2[i].x,
                q1[i].y*q2[i].x,
                q2[i].x,
                q1[i].x*q2[i].y,
                q1[i].y*q2[i].y,
                q2[i].y,
                q1[i].x        ,
                q1[i].y        ,
                1              ;
    }

//    std::cout << "Q: " << std::endl;
//    std::cout << Q << std::endl;

    Eigen::FullPivHouseholderQR<MatrixXd> QRofQ;
    QRofQ.compute(Q.transpose());
    MatrixXd QR = QRofQ.matrixQ();

//    std::cout << "QR: " << std::endl << QR << std::endl;

    Eigen::JacobiSVD<MatrixXd> svd(Q, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
//    std::cout << "V: " << std::endl << svd.matrixV() << std::endl;

    TriPoly1 Epoly[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            int iE = 3*r+c;
            int iQ = 3*c+r;
            double x = V(iQ,5);
            double y = V(iQ,6);
            double z = V(iQ,7);
            double w = V(iQ,8);
            Epoly[iE][0] = x;
            Epoly[iE][1] = y;
            Epoly[iE][2] = z;
            Epoly[iE][3] = w;
            X(c,r) = x;
            Y(c,r) = y;
            Z(c,r) = z;
            W(c,r) = w;
        }
    }

    // compute E*E^T
    TriPoly2 EETpoly[9];
    for (int i = 0; i < 3; i++) { // row
        for (int j = 0; j < 3; j++) { // column
            TriPoly2* curr = &EETpoly[3*i+j];
            curr->zero();
            for (int k = 0; k < 3; k++)
                *curr += multTriPoly(Epoly[3*i+k],Epoly[3*j+k]);
        }
    }

    // compute trace(E*E^T)
    TriPoly2 tr = EETpoly[0];
    tr += EETpoly[4];
    tr += EETpoly[8];

    // compute E*E^T - 0.5*I*tr(E*E^T)
    tr *= -0.5;
    EETpoly[0] += tr;
    EETpoly[4] += tr;
    EETpoly[8] += tr;

    // compute F = (E*E^T - 0.5*I*tr(E*E^T) ) *E
    for (int i = 0; i < 3; i++) { // row
        for (int j = 0; j < 3; j++) { // column
            TriPoly3* curr = &F[3*i+j];
            curr->zero();
            for (int k = 0; k < 3; k++) {
                *curr += multTriPoly(EETpoly[3*i+k],Epoly[3*k+j]);
            }
        }
    }

    // add tenth constraint: det(E) = 0
    {
        TriPoly2 m1, m2, diff;
        m1 = multTriPoly(Epoly[3*1+1],Epoly[3*2+2]);
        m2 = multTriPoly(Epoly[3*1+2],Epoly[3*2+1]);
        diff = m1 - m2;
        F[9] =  multTriPoly(diff,Epoly[3*0+0]);

        m1 = multTriPoly(Epoly[3*1+2],Epoly[3*2+0]);
        m2 = multTriPoly(Epoly[3*1+0],Epoly[3*2+2]);
        diff = m1 - m2;
        F[9] += multTriPoly(diff,Epoly[3*0+1]);

        m1 = multTriPoly(Epoly[3*1+0],Epoly[3*2+1]);
        m2 = multTriPoly(Epoly[3*1+1],Epoly[3*2+0]);
        diff = m1 - m2;
        F[9] += multTriPoly(diff,Epoly[3*0+2]);
    }
}

void FivePointSolver::fivePointNisterGroebner(const std::vector<cv::Point2d>& q1, const std::vector<cv::Point2d>& q2, std::vector<Matrix3d>& E)
{
    // TODO: add switch for enabling normalization

    assert(q1.size() == 5);
    assert(q2.size() == 5);

//    for (int i = 0; i < 5; i++) {
//        cout << q1[i].x << ", " << q1[i].y << ", " << q2[i].x << ", " << q2[i].y << ";" << endl;
//    }

    TriPoly3 F[10];
    Matrix3d X, Y, Z, W;
    computeConstraints(q1,q2,F,X,Y,Z,W);

//    cout << "X Y Z W:" << endl << X << endl << Y << endl << Z << endl << W << endl;

    // build equation system A: left half is A1, right half A2
    Matrix<double,10,10> A1;
    Matrix<double,10,10> A2;
    for (int i = 0; i < 10; i++) {
        TriPoly3& a = F[i];
        // Stewenius' order:
        A1.row(i) <<  a[0],  a[4], a[13],  a[1], a[8], a[16],  a[5], a[10], a[14], a[2];
        A2.row(i) << a[12], a[19],  a[9], a[18], a[17], a[6],  a[7], a[11], a[15], a[3];
    }
//    cout << "A1:" << endl << A1 << endl;
//    cout << "A2:" << endl << A2 << endl;

    FullPivLU<Matrix<double, 10, 10> > lu(A1);
    Matrix<double,10,10> A;
    A = lu.solve(A2);

    //cout << "A:" << endl << A << endl;

    double a1 = A(0,0);
    if (a1 != a1) {
        // is NaN, can't get a valid E from here
        return;
    }

    // here comes the groebner basis trick
    Matrix<double, 10, 10, RowMajor> M = MatrixXd::Zero(10,10);
    M.row(0) = -A.row(0);
    M.row(1) = -A.row(1);
    M.row(2) = -A.row(2);
    M.row(3) = -A.row(4);
    M.row(4) = -A.row(5);
    M.row(5) = -A.row(7);
    M(6,0) = 1;
    M(7,1) = 1;
    M(8,3) = 1;
    M(9,6) = 1;

//    std::cout << "M: " << endl << M << endl;

    Eigen::EigenSolver<Matrix<double, 10, 10, RowMajor> > es(M);

//    std::cout << "eigenvectors:" << endl
//                 << es.eigenvectors() << endl;

    Eigen::EigenSolver<Matrix<double, 10, 10, RowMajor> >::EigenvectorsType ev = es.eigenvectors();

    // for each of the 10 complex solutions
    for (int i = 0; i < 10; i++) {
        // solve polynomial in x y z w, set w = 1;
        std::complex<double> w, x, y, z;
        w = ev(9,i);

        x = ev(6,i) / w;
        y = ev(7,i) / w;
        z = ev(8,i) / w;

        if (x.imag() == 0.0 && y.imag() == 0.0 && z.imag() == 0.0) {
            // real solution:
            Matrix3d Ecurr = (x.real()*X + y.real()*Y + z.real()*Z + W).transpose();
            Ecurr /= Ecurr.norm();
//            std::cout << "solution: " << x.real() << ", " << y.real() << ", " << z.real() << ", "
//                      << "E: " << std::endl << Ecurr << std::endl;

            for (int p = 0; p < 5; p++) {
                double err = Eigen::Vector3d(q1[p].x, q1[p].y, 1.0).transpose()
                        *Ecurr* Eigen::Vector3d(q2[p].x, q2[p].y, 1.0);
                assert(fabs(err) < 1e-8);
            }
            E.push_back(Ecurr);
        }
    }
}

void FivePointSolver::triangulate(const std::vector<cv::Point2d>& q1,
                                  const std::vector<cv::Point2d>& q2,
                                  const Matrix3d& R,
                                  const Vector3d& T,
                                  std::vector<Vector3d>& X)
{
    // See Hartley, Zisserman  p.312.
    // Note that P in this case is the identity and P' is [R, T]
    Matrix<double,3,4> P;
    P << R, T;
    X.resize(q1.size());

    for (unsigned int i = 0; i < q1.size(); i++) {
        Matrix4d A;
        A.row(0) = (Matrix<double,1,4>() << -1,0,q1[i].x,0).finished();
        A.row(1) = (Matrix<double,1,4>() << 0,-1,q1[i].y,0).finished();
        A.row(2) = q2[i].x * P.row(2) - P.row(0);
        A.row(3) = q2[i].y * P.row(2) - P.row(1);

        // TODO: implement a faster 4x4 SVD decomposition manually, e.g. as proposed by Nister
        Eigen::JacobiSVD<Matrix4d> svd(A,Eigen::ComputeFullV | Eigen::ComputeFullU);
        Matrix4d V = svd.matrixV();
        Vector4d Xh = V.col(3);
        double w = Xh.w();
        X[i] = Vector3d(Xh.x()/w,Xh.y()/w,Xh.z()/w);
    }
}

} // namespace
