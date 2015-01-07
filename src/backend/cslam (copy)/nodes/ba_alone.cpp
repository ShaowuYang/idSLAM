#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <cs_geometry/Camera.h>
#include <cs_geometry/FivePointSolver.h>

#include <registration/RegistratorSIM3.h>
#include <registration/Registrator3P.h>
#include <registration/Registrator5P.h>
#include <registration/RegistratorBAPTAM.h>
#include <registration/RegistratorKFs.h>

#include <tools/Profiler.h>
#include <slam/Keyframe.h>

using namespace cs_geom;
using namespace cslam;
using namespace std;

double validDepth = 0.3;

int main(int argc, char **argv)
{
    Sophus::SE3d relPose1T0;

    if (argc != 4) {
        cout << "usage: " << argv[0] << " camera.yaml kf0 kf1" << endl;
        return EXIT_FAILURE;
    }

    cs_geom::Camera cam(argv[1]);
    if (!cam.isGood()) {
        cerr << "Could not open camera: " << argv[1] << endl;
    }

    Profiler* prof = Profiler::Instance();

    string kf0file = argv[2];
    string kf1file = argv[3];

    Keyframe kf0, kf1;

    std::ifstream ifs(kf0file.c_str(), std::ios::binary);
    boost::archive::binary_iarchive ia0(ifs);
    ia0 >> kf0;
    ifs.close();

    ifs.open(kf1file.c_str(), std::ios::binary);
    boost::archive::binary_iarchive ia1(ifs);
    ia1 >> kf1;
    ifs.close();

    Sophus::SE3d relPose0T1PTAM;

    cout << "KF0: mappoints " << kf0.mapPoints.size() << ", kpts: " << kf0.keypoints.size() << endl;
    cout << "KF1: mappoints " << kf1.mapPoints.size() << ", kpts: " << kf1.keypoints.size() << endl;;

    if (kf0.edges.find(kf1.id) != kf0.edges.end()) {
        Edge edge = *kf0.edges[kf1.id];
        cout << "edge: " << kf0.id << " -> " << kf1.id << endl;
        cout << "type: " << edge.type << " valid: " << edge.valid << endl
             << edge.aTb.log().transpose() << endl
             << edge.aTb.matrix() << endl;
        relPose0T1PTAM = edge.aTb;
    }

    if (kf1.edges.find(kf0.id) != kf1.edges.end()) {
        Edge edge = *kf1.edges[kf0.id];
        cout << "edge: " << kf1.id << " -> " << kf0.id << endl;
        cout << "type: " << edge.type << " valid: " << edge.valid << endl
             << edge.aTb.log().transpose() << endl
             << edge.aTb.matrix() << endl;
        relPose0T1PTAM = edge.aTb.inverse();
    }


    RegistratorKFs reg_kf(cam);

    prof->begin("fullreg");
    boost::shared_ptr<Edge> edge = reg_kf.tryAndMatch(kf0, kf1);
    prof->end("fullreg");
    if (edge) {
        cout << "### EDGE" << endl;
        cout << "aTb: " << endl;
        cout << edge->aTb.matrix() << endl;
    } else {
        cout << "### FAILED TO REGISTER" << endl;
    }


    boost::scoped_ptr<cv::DescriptorMatcher> matcher(new cv::BFMatcher(cv::NORM_HAMMING2, false));

    std::vector<cv::DMatch> matches01, matches10;
    std::vector<cv::DMatch> matches01in, matches10in;
    std::vector<Observation> obs01, obs10;
    cv::Mat matchImg;

    double thresholdPx = 3.0;


    prof->begin("match");
    matcher->match(kf0.mpDesc, kf1.kpDesc, matches01);
    prof->end("match");
    cv::drawMatches(kf0.rgbImage, kf0.mpKeypoints, kf1.rgbImage, kf1.keypoints, matches01, matchImg);
    cv::imshow("matches", matchImg);
    cv::waitKey(0);

    prof->begin("reg");
    Registrator3P reg(cam);
    Sophus::SE3d relPose01 = reg.solve(kf0, kf1, matches01);
    matches01in = reg.getInliers(kf0, kf1, matches01, relPose01, thresholdPx, obs01);
    prof->end("reg");
    std::cout << "inliers: " << matches01in.size() << std::endl;

    cv::drawMatches(kf0.rgbImage, kf0.mpKeypoints, kf1.rgbImage, kf1.keypoints, matches01in, matchImg);
    cv::imshow("matches", matchImg);
    cv::waitKey(0);

    prof->begin("match");
    matcher->match(kf1.mpDesc, kf0.kpDesc, matches10);
    prof->end("match");
    cv::drawMatches(kf1.rgbImage, kf1.mpKeypoints, kf0.rgbImage, kf0.keypoints, matches10, matchImg);
    cv::imshow("matches", matchImg);
    cv::waitKey(0);

    prof->begin("reg");
    Sophus::SE3d relPose10 = reg.solve(kf1, kf0, matches10);
    matches10in = reg.getInliers(kf1, kf0, matches10, relPose10, thresholdPx, obs10);
    prof->end("reg");
    std::cout << "inliers: " << matches10in.size() << std::endl;

    cv::drawMatches(kf1.rgbImage, kf1.mpKeypoints, kf0.rgbImage, kf0.keypoints, matches10in, matchImg);
    cv::imshow("matches", matchImg);
    cv::waitKey(0);


    prof->begin("sim3");
    RegistratorSIM3 regSIM(false);
    regSIM.solve(obs01, obs10);
    prof->end("sim3");

    std::ofstream poseOut("relpose.txt");
    poseOut.setf( std::ios::scientific, std:: ios::floatfield );
    poseOut.precision(15);
    poseOut << regSIM.aTb_se3().log() << endl;



    //    Sophus::SE3d prior0T1, relPose0T1;
    //    {
    //        prof->begin("ransac");
    //        Registrator5P fivePoints(cam);
    //        fivePoints.solve(kf0.keypoints, kf0.kpDepth, kf1.keypoints, kf1.kpDepth, matchesAll, prior0T1);
    //        matchesInl = fivePoints.inliers();
    //        prior0T1 = fivePoints.relPose0T1();
    //        relPose0T1 = prior0T1;
    //        prof->end("ransac");


    //        FivePointHypothesis h;
    //        Sophus::SE3d Pd = relPose0T1PTAM.inverse(); // P' = 1T0
    //        Eigen::Vector3d t = Pd.translation();
    //        Eigen::Matrix3d S;
    //        S << 0, -t[2], t[1],
    //                t[2], 0, -t[0],
    //                -t[1], t[0], 0;
    //        h.E = (S*Pd.rotationMatrix());
    //        h.E /= h.E.norm();
    //        std::cout << "PTAM's E: " << std::endl << h.E << std::endl;
    //        h.score = 0.0;
    //        for (int i = 0; i < matchesAll.size(); i++) {
    //            int i1 = matchesAll[i].trainIdx;
    //            int i0 = matchesAll[i].queryIdx;
    //            cv::Point2d q0(kf0.keypoints[i0].pt.x, kf0.keypoints[i0].pt.y);
    //            cv::Point2d q1(kf1.keypoints[i1].pt.x, kf1.keypoints[i1].pt.y);

    //            h.score += h.scorePair(cam.unprojectPixel(q0), cam.unprojectPixel(q1));
    //        }
    //        std::cout << "PTAM's score: " << h.score << std::endl;
    //    }

    //    cout << "inliers after ransac: " << matchesInl.size() << endl;
    //    cv::drawMatches(kf0.rgbImage, kf0.keypoints, kf1.rgbImage, kf1.keypoints, matchesAll, matchImg);
    //    cv::imshow("matches", matchImg);
    //    cv::waitKey(0);

    //    {
    //        cout << "prior: " << endl << prior0T1.log().transpose() << endl;
    //        cout << "orientation: " << endl << prior0T1.so3().log().transpose() << endl;
    //        RegistratorBAPTAM ba(cam);
    //        prof->begin("ba_ptam");
    //        ba.solve(kf0.keypoints, kf0.kpDepth, kf1.keypoints, kf1.kpDepth, matchesInl, prior0T1);
    //        prof->end("ba_ptam");
    //        matchesInl = ba.inliers();

    //        cout << "relPose: " << endl << ba.relPose0T1().log().transpose() << endl;
    //        relPose0T1 = ba.relPose0T1();

    //        cout << "inliers after BA: " << matchesInl.size() << endl;
    //        cout << "with depth: " << ba.nInliersDepth() << endl;
    //        cv::drawMatches(kf0.rgbImage, kf0.keypoints, kf1.rgbImage, kf1.keypoints, matchesInl, matchImg);
    //        cv::imshow("matches", matchImg);
    //        cv::waitKey(0);
    //    }

    //    prof->status();

    //    std::ofstream poseOut("relpose.txt");
    //    poseOut.setf( std::ios::scientific, std:: ios::floatfield );
    //    poseOut.precision(15);
    //    poseOut << relPose0T1.log() << endl;

    return 0;
}
