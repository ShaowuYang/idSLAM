#include "SLAMSystem.h"

#include <fenv.h>
#include <queue>

#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <cs_geometry/Conversions.h>
#include <ptam/MapPoint.h>

using namespace cs_geom;
using namespace backend;
using namespace std;

SLAMSystem::SLAMSystem(ptam::Map &m, const boost::scoped_ptr<cs_geom::Camera> * cam, const std::string& vocFile, bool closeLoops, bool saveKeyframes, bool pubmap)
    : map_(m),
      pgo_(),
      closeLoops_(closeLoops),
      saveKeyframes_(saveKeyframes),
      pubmap_(pubmap)
{
    // TODO: resolve those initialization parameters
//    cslamTptam_ = bodyTcam;
    for (int i = 0; i < AddCamNumber + 1; i ++)
        cam_[i] = *cam[i];
    bodyTcam_ = Sophus::SE3();
    cslamTptam_ = Sophus::SE3d();
    loopDetector_.reset(new LoopDetector(cam_, vocFile));
//    matcher_.reset(new BruteForceMatcher(cam, AddCamNumber + 1));
    registrator_.reset(new RegistratorKFs(cam_, AddCamNumber + 1));

    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

    GMapUpdated = false;
    maxkfInied = false;
    debugmarkLoopDetected = false;

    reset();
}

void SLAMSystem::reset()
{
    cslamTptam_ = bodyTcam_;

    pgo_.reset();
    loopDetector_->reset();

    keyframes_.clear();
    keyframes_add_.clear();
    ptam_edges_.clear();
    ptam_edges_local_.clear();
    wlKeyFrames.clear();
    wlEdges.clear();
    keyframeUpdatedPoses.clear();

    latestKFinQueue_ = -1;
    latestKFinMap_ = -1;
}

void SLAMSystem::addEdges()
{
    boost::unique_lock<boost::mutex> lock(mutex_wl);// write access
    while (wlEdges.size()>0) {
        boost::shared_ptr<ptam::Edge> edge = wlEdges[0];
        wlEdges.erase(wlEdges.begin());

        // add new edge to local edges
        assert(edge->idA <= keyframes_.size());
        // add them to the ptam_edges, also consider overwriting issue.
        bool existed = false;
        int iover = 0;
        if (abs(edge->idB-edge->idA)==1){
            // check whether existed already
            for (int i = 0; i < ptam_edges_.size(); i ++)
                if (edge->idA == ptam_edges_[i]->idA && edge->idB == ptam_edges_[i]->idB){
                    existed = true;
                    iover = i;
                    break;}
        }
        if (existed)// ignore it since will be updated by pgo
            continue;
        ptam_edges_.push_back(edge);

        //There might be multiple edges between two kfs, if there are multi-kfs in the waiting list
        existed = false;
        iover = 0;
        if (abs(edge->idB-edge->idA)==1){
            // check whether existed already
            for (int i = 0; i < ptam_edges_local_.size(); i ++)
                if (edge->idA == ptam_edges_local_[i]->idA && edge->idB == ptam_edges_local_[i]->idB){
                    existed = true;
                    iover = i;
                    break;}
        }
        if (!existed)
            ptam_edges_local_.push_back(edge);
        else// overwrite the existing ones
            ptam_edges_local_[iover] = edge;
    }
    cout << "Edges added to the backend: " << ptam_edges_local_.size() << endl;

    lock.unlock();
}

// add all kfs in the waiting list to the global map
// normally there should be only one kf to be added, unless the backend is tooooo slow
void SLAMSystem::addKeyframes()
{
    boost::unique_lock<boost::mutex> lock(mutex_wl);// write access
    boost::shared_ptr<ptam::KeyFrame> kf; // (new ptam::KeyFrame)
//    boost::unique_lock<boost::mutex> lock(syncMutex);

    pgoRunRequired = false;//
    // pgo for a whole loop or a local window?
    // this is a double window inspired idea
    pgoLoopRequired = false;
    bool localoopGot = false;
    bool loopGot = false;// find a loop already? then skip other kfs
    debugmarkLoopDetected = false;

    std::vector<boost::shared_ptr<ptam::KeyFrame> > wlkfs;
    while (wlKeyFrames.size() > 0){
        wlkfs.push_back(map_.vpKeyFrames[wlKeyFrames[0]]); // TODO: multiple cameras
        wlKeyFrames.erase(wlKeyFrames.begin());
    }
    lock.unlock();

    // add all keyframes in kfs, and clear it
    /// TODO： if there's kfs from additional cams, add them
    while (wlkfs.size() > 0)    {
        kf = wlkfs[0];
        wlkfs.erase(wlkfs.begin());

        assert(kf->id == keyframes_.size());// Not added yet. strong assumption

        // TODO: only do pgo when we have enough kfs (max kfs in the local map)
        if (kf->id >= maxKfsInLMap-1) {// && kf->haveGroundData
            pgoRunRequired = true;
        }

        // TODO: when PTAM send this KF without giving us an outgoing edge before.
        /*/ This happens whenever the map is reset due to tracking failure.
        if(false && ptam_edges_.size() <= kf->id) {
            // Make sure there is really only one edge missing:
            assert(ptam_edges_.size() == kf->id);

            // Create this "motion model" edge: We cannot know the transform yet,
            // need to wait for the next keyframe to compute it.
            boost::shared_ptr<ptam::Edge> edge(new ptam::Edge);
            edge->idA = kf->id;
            edge->idB = kf->id + 1;
            edge->type = ptam::EDGE_MOTIONMODEL;
            edge->valid = false;
            ptam_edges_.push_back(edge);
        }*/
        /*/ TODO: Check whether there is a "motion model" edge waiting to be completed:
        if (false && kf->id > 0 && !ptam_edges_[kf->id - 1]->valid) {
            // yes, indeed
            boost::shared_ptr<ptam::Edge> we = ptam_edges_[kf->id - 1];
            we->aTb = cs_geom::toSophusSE3(keyframes_[kf->id - 1]->se3CfromW*kf->se3CfromW.inverse());
            we->valid = true;

            pgoRunRequired = true;
        }*/

        ///////////////////
        keyframes_.push_back(kf);
        pgo_.addKeyframe(*kf);
        // add kf_additional only aside with cam1
        for (int cn = 0; cn < AddCamNumber; cn ++)
            if (map_.vpKeyFramessec[cn].size() >= kf->id){
                boost::shared_ptr<ptam::KeyFrame> kf(new ptam::KeyFrame);
                kf = map_.vpKeyFramessec[cn][kf->id];
                keyframes_add_.push_back(kf);
            }

        latestKFinMap_ = kf->id;

        // The actual work starts here: Finalize keyframe data.
        kf->finalizeKeyframeBackend();

        /*if (saveKeyframes_) {
            std::ofstream ofs((std::string("kf")+boost::lexical_cast<std::string>(kf->id)).c_str() , std::ios::out | std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << *kf;
            ofs.close();
        }*/

        /// TODO: detect loops with the additional kfs
        // Now find some loops.
        // Implicit loops:
        static int freelocalnum = 5;
        static int localfreekfs = freelocalnum;
        int bestMatch = findBestKeyframeForMatching(*kf);
        if ((localfreekfs >= freelocalnum) && bestMatch > -1) {
            std::cout << "best match: " << bestMatch << std::endl;
            boost::shared_ptr<ptam::Edge> edge = registrator_->tryAndMatch(*kf, *keyframes_[bestMatch]);
            if (edge) {
                std::cout << "successfully matched with atb: \n" << edge->aTb.so3().log().norm()*180./M_PI
                          << "\n" << edge->aTb.translation().norm() << "new edge got." << std::endl;
                kf->edges.insert(make_pair(edge->idA, edge));
                // TODO: add rotation item
                double dist = edge->aTb.translation().norm();
                kf->neighbor_ids_ordered_by_distance.insert(make_pair(dist, edge->idA));

                double scendepth = (kf->dSceneDepthMean + keyframes_[bestMatch]->dSceneDepthMean) / 2.0;
                pgo_.addEdge(*edge, true, scendepth);
                cout << "local loop edge added to pg." << endl;
                pgoRunRequired = true;
                localoopGot = true;
                pgoLoopRequired = true; // local loop also effect the whole loop
                debugmarkLoopDetected = true;
                localfreekfs = 0;
                loopGot = true;// no appearance loop detection if metric loop detected
            }
        }
        localfreekfs ++;
        if (localfreekfs > freelocalnum+1) localfreekfs = freelocalnum+1;

        // Explicit appearence loop closure detection:
        static int freenum = 5;
        static int loopfreekfs = freenum; // free some kfs after a loop is detected
        if (!loopGot && closeLoops_ && (loopfreekfs >= freenum)) {
            std::cout << "Detecting large loop closure ..." << std::endl;
            // TODO: implement new LD method using online vocabulary
            int loopMatch = loopDetector_->detectLoop(*kf);
            if (loopMatch > 0) {
                boost::shared_ptr<ptam::Edge> edge = registrator_->tryAndMatchLargeLoop(*kf, *keyframes_[loopMatch]);
                std::cout << "loop closure match: " << loopMatch << std::endl;
                if (edge) {
                    std::cout << "successfully matched with atb: \n" << edge->aTb.rotationMatrix()
                              << "\n" << edge->aTb.translation() << "new edge got." << std::endl;
                    kf->edges.insert(make_pair(edge->idA, edge));
                    double dist = edge->aTb.translation().norm();
                    kf->neighbor_ids_ordered_by_distance.insert(make_pair(dist, edge->idA));

                    double scendepth = (kf->dSceneDepthMean + keyframes_[loopMatch]->dSceneDepthMean) / 2.0;
                    pgo_.addEdge(*edge, true, scendepth);

                    pgoRunRequired = true;
                    loopGot = true;
                    pgoLoopRequired = true;
                    debugmarkLoopDetected = true;
                    loopfreekfs = 0;
                }
            }
        }

        loopfreekfs ++;
        if (loopfreekfs > freenum+1) loopfreekfs = freenum+1;
    }
    cout << "Keyframe added to the PG: "<< keyframes_.size() << endl;

    // add all edges to those kfs after all kfs have been added
    // for convinence of neighbor searching in defining sub-g, edges of a kf point back, not forward
    // edges added in loop closure must be consistent with this definition
    for (int i = 0; i < ptam_edges_local_.size(); i ++){
        const boost::shared_ptr<ptam::Edge> edge = ptam_edges_local_[i];
        boost::shared_ptr<ptam::KeyFrame> ekf = keyframes_[ptam_edges_local_[i]->idB];

        if (ekf->edges.find(edge->idA) != ekf->edges.end()){
            // keep the existing edges? or just replace them? replace them
            for (ptam::KeyFrame::EdgeMap::iterator it = ekf->edges.begin(); it != ekf->edges.end(); it ++){
                if (it->first != edge->idA)
                    continue;
                it->second = edge;
            }
        }
        else{
            ekf->edges.insert(make_pair(edge->idA, edge));

            double dist = edge->aTb.translation().norm();
            ekf->neighbor_ids_ordered_by_distance.insert(make_pair(dist, edge->idA));
        }
    }
    for (int ee = ptam_edges_local_.size()-1; ee >=0; ee-- )
        pgo_.addEdge(*ptam_edges_local_[ee], true, ptam_edges_local_[ee]->dSceneDepthMean);
    ptam_edges_local_.clear();

    // If this is an isolated new keyframe (e.g. because ptam reset itself),
    // try to match it with the one before.
//    int nOdom = countGoodOdometryEdges();
//    if (nOdom == 0 && kf->id > 0) {
//        cout << "MOTION MODEL EDGE" << endl;
//        // TODO: RANSAC and PnP match should be done in the front end
//        boost::shared_ptr<Edge> edge = registrator_->tryAndMatch(*kf, *keyframes_[kf->id - 1]);
//        if (edge) {
//            std::cout << "successfully matched with atb:" << std::endl;
//            kf->edges.insert(make_pair(edge->idB, edge));
//            double dist = sqrt(double(edge->aTb.translation().transpose()*edge->aTb.translation()));
//            kf->neighbor_ids_ordered_by_distance.insert(make_pair(dist, edge->idB));

//            pgo_.addEdge(*edge);
//            pgoRunRequired = true;
//        }
//    }

    if (keyframes_.size() < maxKfsInLMap-1)
        pgoRunRequired = false;
    cout << "Edges added to the PG with loopdetected = " << pgoLoopRequired<< ", " << localoopGot << ", " << pgoRunRequired << endl;
}

void SLAMSystem::runPGO()
{
    kfinpgo = 0;
    if (pgoRunRequired) {
        cout << "Doing PGO..." << endl;
        // TODO: if no appearance loop detected, only do local pgo, otherwise, do pgo within the whole loop
        // todo so, we need to change the the way the optimiser add verteces and edges
        if (!pgoLoopRequired){// do pgo within a defined sub-graph
            std::set<int> vids;
            defineSubGraph(vids, 5*maxKfsInLMap);
            kfinpgo = vids.size();

//            pgo_.optimize_portion(vids);
            vids.clear();
        }
        else
        {// true uniform-cost search to define the adaptive window
            std::set<int> vids;
            defineSubGraph(vids, 500*maxKfsInLMap);
            kfinpgo = vids.size();

            pgo_.optimize_portion(vids);
            vids.clear();

            // write access
            boost::unique_lock<boost::mutex> lock(syncMutex);
            pgo_.applyResult(keyframes_, keyframeUpdatedPoses);

            setGMapUpdated(true);
            lock.unlock();
        }
//            pgo_.optimize();
        cout<< "finished pgo..." << endl;

        // based on updated pose graph: update correction cslamTptam
//        const ptam::KeyFrame& latestKF = *keyframes_[latestKFinMap_];
//        cslamTptam_ = latestKF.se3CfromW.inverse()*latestKF.se3CfromW;

        pgoRunRequired = false;
        pgoLoopRequired = false;
        cout << "Done PGO." << endl;
    }
}

void SLAMSystem::defineSubGraph(std::set<int>& vids, int vmaxnum)
{
    // define the subgraph to be optimised according to covisibility
    // this needs the map points in the backend also remember which kfs measured them, which needs to be updated all the time, and project each map point to the kfs
    // now, to be more efficient, simply chose those closed connected kfs, according to kfs distance,
    // in a simplified uniform-cost search manner, while arranging leaves with a accent manner
    vmaxnum = min(vmaxnum, (int)keyframes_.size());
    queue<int> neib_queue;
    boost::shared_ptr<ptam::KeyFrame> kf = keyframes_.back();
    neib_queue.push(kf->id);

    while (vids.size() < vmaxnum && neib_queue.size()!=0){
        int leaf = neib_queue.front();
        neib_queue.pop();

        if (vids.find(keyframes_[leaf]->id)!=vids.end())
            continue;
        vids.insert(leaf);

        for (multimap<double, int>::const_iterator it = keyframes_[leaf]->neighbor_ids_ordered_by_distance.begin();
             it != keyframes_[leaf]->neighbor_ids_ordered_by_distance.end();  it++)
            neib_queue.push(it->second);
    }

    cout << "sub graph defined with size: " << vids.size() << endl;
//    for (set<int> ::iterator i = vids.begin(); i != vids.end(); i ++)
//        cout << *i << endl;
}

int SLAMSystem::countGoodOdometryEdges()
{
    int n = 0;
    for (int i = keyframes_.back()->id - 1; i >= 0; i--) {
        if (ptam_edges_[i]->type != ptam::EDGE_PTAM || !ptam_edges_[i]->valid) {
            //  this breaks the chain of good edges
            break;
        } else {
            // good edge, increases chain
            n++;
        }
    }
    return n;
}

// TODO: local loop closure could consider scendepth inform.
int SLAMSystem::findBestKeyframeForMatching(const ptam::KeyFrame& kf)
{
    int bestInd = -1;
    int bestVisible = 0;

    BOOST_FOREACH(boost::shared_ptr<ptam::KeyFrame> pkfi, keyframes_) {
        const ptam::KeyFrame& kfi = *pkfi;

        // Enforce lower bound for temporal distance
        // This value should be at least as high as ptam's window size
        if (std::abs(kf.id - kfi.id) < 5)
            continue;

        // Enforce low translational distance
        Eigen::Vector3d posdiff = cs_geom::toSophusSE3(kf.se3CfromW).translation() - cs_geom::toSophusSE3(kfi.se3CfromW).translation();
        if (posdiff.norm() > kf.dSceneDepthMean/4.0)
            continue;

        // Enforce low angular distance
        Sophus::SE3d relPose = cs_geom::toSophusSE3(kf.se3CfromW * kfi.se3CfromW.inverse());
        if (relPose.so3().log().norm() > 25.*M_PI/180.)
            continue;

        // This is a serious candidate: Project all map points
        // TODO: could consider to normalize it using mean scene depth.
        int nVisible = 0;
        BOOST_FOREACH(boost::shared_ptr<ptam::MapPoint> p, kfi.mapPoints) {
            nVisible += cam_[p->nSourceCamera].isVisible(relPose*cs_geom::toEigenVec(p->v3RelativePos)); /// use my relative pose to its father kf
        }
        BOOST_FOREACH(boost::shared_ptr<ptam::MapPoint> p, kf.mapPoints) {
            nVisible += cam_[p->nSourceCamera].isVisible(relPose.inverse()*cs_geom::toEigenVec(p->v3RelativePos));
        }

        if (nVisible > bestVisible) {
            bestInd = kfi.id;
            bestVisible = nVisible;
        }
    }

    return bestInd;
}
