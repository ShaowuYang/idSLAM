// Copyright 2008 Isis Innovation Limited
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include <fstream>
#include <iostream>
#include <gvars3/instances.h>
#include <boost/filesystem/operations.hpp>

using namespace GVars3;
using namespace ptam;

Map::Map()
{
    Reset();
}

void Map::Reset()
{
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mutex);
    vpPoints.clear();
    bGood = false;
    didFullBA = false;

    erasedAllCallback = &emptyErasedAllCb;
//    erasedKfCallback = &emptyErasedKfCb;

    lock.unlock();
}

void Map::EraseBadPoints(bool performLock)
{
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock;
    if(performLock)
    	lock = boost::unique_lock<boost::shared_mutex>(mutex);
    
    int nBad = 0;
    for(int i = vpPoints.size()-1; i>=0; i--)
    {
        if(vpPoints[i]->bBad)
        {
            vpPoints.erase(vpPoints.begin() + i);
            nBad++;
        }
    };
    if(performLock)
	    lock.unlock();
}

void Map::EraseOldKeyFrames(bool performLock) {
	// write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock;
    if(performLock)
    	lock = boost::unique_lock<boost::shared_mutex>(mutex);
    
    static gvar3<int> maxKeyFrames("MapMaker.RecentWindowSize", "4", SILENT);
    static gvar3<int> gvnPublishKF("MapMaker.PublishKF", 1, SILENT);

    std::cout << "Try erasing keyframes..." << std::endl;
    // in dual camera case, the association number should also be corrected
    // keep all necceceary second cam kfs for the back end,
    // so the first cam kf may be more than maxKeyFrames
    while(vpKeyFramessec.size() > *maxKeyFrames) {// Erase oldest keyframes

        // Erase dependent points if they can not be observed by the new keyframes
        // (all recent maxKeyFrames kfs)which are not to be removed now, and keep those can be observed, and
        // hand over the source kf identity to a new kf
        // this requires each map point to record all kfs measuring it, to speed up the search
        for(int i = vpPoints.size()-1; i >= 0; i--) {
            boost::shared_ptr<MapPoint> vpPoint = vpPoints[i];
            if((vpPoint->nSourceCamera==1) && (vpKeyFramessec.front() == vpPoint->pPatchSourceKF.lock())){
                bool obsable = false;
                int kfnum = std::min(*maxKeyFrames, (int)vpKeyFramessec.size());
                // if cannot be observed by kept kfs, remove it, otherwise keep it
                for (int j = kfnum-1; j > 0; j-- ){
                    if(vpPoint->MMData.sMeasurementKFs.count(vpKeyFramessec[j])){
                        obsable = true;

                        // update all info useful related to this point
                        vpPoint->pPatchSourceKF = vpKeyFramessec[j];
                        vpPoint->v3SourceKFfromeWorld = vpKeyFramessec[j]->se3CfromW;
                        vpPoint->v3RelativePos = vpKeyFramessec[j]->se3CfromW * vpPoint->v3WorldPos;
                        for(meas_it jiter = vpKeyFramessec[j]->mMeasurements.begin(); jiter!= vpKeyFramessec[j]->mMeasurements.end(); jiter++)
                            if (vpPoint == jiter->first){// if this operation not allowed, use: set
                                vpPoint->nSourceLevel = jiter->second.nLevel;

                                // Patch source stuff, leave the refresh step to the tracker
                                int px = jiter->second.v2RootPos[0] / (1 << vpPoint->nSourceLevel);
                                int py = jiter->second.v2RootPos[1] / (1 << vpPoint->nSourceLevel);
                                vpPoint->irCenter = CVD::ImageRef(px, py);
                            }

                        vpPoint->bfixed = true;
                        vpPoint->sourceKfIDtransfered = true;
                        vpPoint->refreshed = false;

                        // erase the measurementKF info for this point
                        if(vpPoint->MMData.sMeasurementKFs.count(vpKeyFrames[0]))
                            vpPoint->MMData.sMeasurementKFs.erase(vpKeyFramessec[0]);

                        break;
                    }
                }
                if (!obsable)
                    vpPoints.erase(vpPoints.begin() + i);
            }
        }

        // remove or adjust kf association in multi-camera case if appliable
        if (vpKeyFramessec[0]->mAssociateKeyframe)
            vpKeyFrames[vpKeyFramessec[0]->nAssociatedKf]->mAssociateKeyframe = false;
        // and other kfs of the first cam
        for (int i = 0; i < vpKeyFrames.size(); i ++)
            if ( vpKeyFrames[i]->mAssociateKeyframe)
                vpKeyFrames[i]->nAssociatedKf --;

        vpKeyFramessec.erase(vpKeyFramessec.begin());
        std::cout<<"!!Sec cam keyframes removed!" << vpKeyFramessec.size() << "\n";
    }

    while(vpKeyFrames.size() > *maxKeyFrames) {// Erase oldest keyframe

        // Erase dependent points if they can not be observed by the new keyframes
        // (all recent maxKeyFrames kfs)which are not to be removed now, and keep those can be observed, and
        // hand over the source kf identity to a new kf
        // this requires each map point to record all kfs measuring it, to speed up the search
        for(int i = vpPoints.size()-1; i >= 0; i--) {
            boost::shared_ptr<MapPoint> vpPoint = vpPoints[i];
            if((vpPoint->nSourceCamera== 0) && (vpKeyFrames.front() == vpPoint->pPatchSourceKF.lock())){
                bool obsable = false;
                int kfnum = std::min(*maxKeyFrames, (int)vpKeyFrames.size());
                // if cannot be observed by kept kfs, remove it
                for (int j = kfnum-1; j > 0; j --){
                    if(vpPoint->MMData.sMeasurementKFs.count(vpKeyFrames[j])){
                        obsable = true;

                        // update all info useful related to this point
                        vpPoint->pPatchSourceKF = vpKeyFrames[j];
                        for(meas_it jiter = vpPoint->pPatchSourceKF.lock()->mMeasurements.begin(); jiter!= vpPoint->pPatchSourceKF.lock()->mMeasurements.end(); jiter++)
                            if (vpPoint == jiter->first){// if this operation not allowed, use: set
                                vpPoint->nSourceLevel = jiter->second.nLevel;

                                // Patch source stuff, leave the refresh step to the tracker
                                int px = jiter->second.v2RootPos[0] / (1 << vpPoint->nSourceLevel);
                                int py = jiter->second.v2RootPos[1] / (1 << vpPoint->nSourceLevel);
                                vpPoint->irCenter = CVD::ImageRef(px, py);
                            }

                        vpPoint->sourceKfIDtransfered = true;
                        vpPoint->refreshed = false;

                        // erase the measurementKF info for this point
                        if(vpPoint->MMData.sMeasurementKFs.count(vpKeyFrames[0]))
                            vpPoint->MMData.sMeasurementKFs.erase(vpKeyFrames[0]);

                        break;
                    }
                }
                if (!obsable)
                    vpPoints.erase(vpPoints.begin() + i);
            }
        }

        // remove kf association in multi-camera case if appliable
        if (vpKeyFrames[0]->mAssociateKeyframe){
            vpKeyFramessec[vpKeyFrames[0]->nAssociatedKf]->mAssociateKeyframe = false;
        }
        // and other kfs of the first cam
        for (int i = 0; i < vpKeyFramessec.size(); i ++)
            if ( vpKeyFramessec[i]->mAssociateKeyframe)
                vpKeyFramessec[i]->nAssociatedKf --;

        vpKeyFrames.erase(vpKeyFrames.begin());
        std::cout<<"!First cam keyframes removed!" << vpKeyFrames.size() << "\n";
    }

    if(performLock)
	    lock.unlock();
    std::cout << "Try erasing done." << std::endl;
}


void Map::EraseAll() {
    std::vector<boost::shared_ptr<KeyFrame> > erased = vpKeyFrames;
    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock = boost::unique_lock<boost::shared_mutex>(mutex);

    vpKeyFrames.clear();
    vpPoints.clear();

    lock.unlock();

    erasedAllCallback(erased);
}

bool Map::SaveMap(const std::string sPath)
{
    boost::filesystem::path path(sPath);

    if (!boost::filesystem::exists(path)) {
        boost::filesystem::create_directory(path);
    }
    // read access: shared lock
    boost::shared_lock< boost::shared_mutex > lock(mutex);


    lock.unlock();
    return true;
}

bool Map::LoadMap(const std::string sPath)
{

    // write access: unique lock
    boost::unique_lock< boost::shared_mutex > lock(mutex);
    lock.unlock();
    return true;
}
