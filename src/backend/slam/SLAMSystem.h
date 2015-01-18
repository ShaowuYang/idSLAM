#ifndef _CSLAM_SLAM_SYSTEM_H_
#define _CSLAM_SLAM_SYSTEM_H_

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <vector>

#include <sophus/sophus.hpp>

#include <loops/LoopDetector.h>
#include <registration/RegistratorKFs.h>

#include <matching/BruteForceMatcher.h>

//#include "Keyframe.h"
#include "PGOptimizer.h"
#include <ptam/Map.h>
#include <ptam/KeyFrame.h>

namespace backend {

class SLAMSystem
{
public:
    SLAMSystem(ptam::Map &m, const boost::scoped_ptr<cs_geom::Camera> * cam, const std::string& vocFile,
               bool closeLoops = true, bool saveKeyframes = false,
               bool pubmap = true);

    void reset();

    void addKeyframes();
    void addEdges();
//    void updatePoints(const boost::shared_ptr<Keyframe> kf);// removed : no more update
//    void updateKfPose(const boost::shared_ptr<Keyframe> kf);
    // define a subgraph to be optimised when no loop is detected
    // @vids: ids of those vertices we want
    void defineSubGraph(std::set<int>& vids, int vmaxnum);
    void runPGO();

    int countGoodOdometryEdges(); // count number of good odometry edges up to latest keyframe

    std::vector<int> wlKeyFrames;  //a list of kfs orgernised by ID, the waiting list to be added by the backend
    std::vector<boost::shared_ptr<ptam::Edge> > wlEdges;  //kfs, the waiting list to be added by the backend
    const std::vector<boost::shared_ptr<ptam::KeyFrame> >& keyframes() { return keyframes_; }
    std::vector<int>& wlkf() { return wlKeyFrames; }
    Sophus::SE3d cslamTptam() { return cslamTptam_; }

    // interactives with the front end
    bool IsGMapUpdated() { return GMapUpdated;}
    void setGMapUpdated(bool update) {GMapUpdated = update;}
    bool maxkfInied;// whether maxKfsInLMap has been inied?
    void IniMaxkfs(int maxkfs)
    {
        maxKfsInLMap = maxkfs;
        maxkfInied = true;}
    bool debugLoopDetected() {
        bool detected = debugmarkLoopDetected;
        debugmarkLoopDetected = false;
        return detected;}
    unsigned int getMapPointSize(){
        unsigned int pointsize = 0;
        for (unsigned int i = 0 ; i < keyframes_.size(); i ++){
            pointsize += keyframes_[i]->mapPoints.size();
        }
        return pointsize;
    }
    unsigned int getCornersSize(){
        unsigned int cornersize = 0;
        boost::shared_ptr<ptam::KeyFrame> kf = keyframes_[keyframes_.size()-1];
        for (unsigned int i = 0 ; i < LEVELS; i ++){
            cornersize += kf->aLevels[i].vMaxCorners.size();
        }
        return cornersize;
    }

    mutable boost::mutex syncMutex;
    mutable boost::mutex mutex_wl; // for waiting list access

    std::vector<boost::shared_ptr<ptam::KeyFrame> > keyframes_; // a ptr copy of the kfs in the map
    std::vector<boost::shared_ptr<ptam::KeyFrame> > keyframes_add_; // kfs from additional cameras
    std::vector<boost::shared_ptr<ptam::Edge> > ptam_edges_; // ptam edges, indexed by kf A index idA
    std::vector<boost::shared_ptr<ptam::Edge> > ptam_edges_local_;// ptam edges in local BA come with the new kf at the same time
    std::vector<Sophus::SE3d> keyframeUpdatedPoses; /// new keyframe poses after PGO

    bool pubmap_;
    int kfinpgo;
    ofstream backinfolog;

protected:
    int findBestKeyframeForMatching(const ptam::KeyFrame &kf);

    ptam::Map& map_;
    cs_geom::Camera cam_[AddCamNumber + 1]; /// in all the backend, we do not name additional cameras seperately
    PGOptimizer pgo_;

//    boost::scoped_ptr<BruteForceMatcher> matcher_;
    boost::scoped_ptr<RegistratorKFs> registrator_;
    boost::scoped_ptr<LoopDetector> loopDetector_;

    bool GMapUpdated;// the graph has been updated by a pgo?
    int maxKfsInLMap;// number of kfs allowed in the local map
    int latestKFinQueue_;
    int latestKFinMap_;

    Sophus::SE3d cslamTptam_;
    Sophus::SE3d bodyTcam_;

    bool closeLoops_;
    bool saveKeyframes_;
    bool pgoRunRequired;
    bool pgoLoopRequired;

    bool debugmarkLoopDetected;// for paper plot where loop detected
};

} // namespace

#endif /* _CSLAM_SLAM_SYSTEM_H_ */
