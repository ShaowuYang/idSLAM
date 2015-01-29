// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"
#include "ShiTomasi.h"
#include "SmallBlurryImage.h"
#include "LevelHelpers.h"
#include <cvd/vision.h>
#include <cvd/fast_corner.h>
#include <gvars3/gvars3.h>
#include <cvd/image_io.h>
#include "Map.h"
#include "MapPoint.h"

using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace ptam;
using namespace TooN;

void KeyFrame::MakeKeyFrame_Lite(BasicImage<byte> &im, int nCam)
{
    bComplete = false;
  // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  
  // First, copy out the image data to the pyramid's zero level.
  aLevels[0].im.resize(im.size());

//    int x1 = 35;
//    int x2 = im.size().x - 20;

  copy(im, aLevels[0].im);

  static int imnum = 1;
  // Then, for each level...
  for(int i=0; i<LEVELS; i++)
    {
      Level &lev = aLevels[i];
      if(i!=0)
	{  // .. make a half-size image from the previous level..
	  lev.im.resize(aLevels[i-1].im.size() / 2);
	  halfSample(aLevels[i-1].im, lev.im);
	}
//      string imname;
//      stringstream num;
//      num << imnum;
//      num << i;
//      imname = num.str();
//      imname += ".jpg";
//      if (imnum <= 1)//save images
//          CVD::img_save(lev.im, imname);

      // .. and detect and store FAST corner points.
      // I use a different threshold on each level; this is a bit of a hack
      // whose aim is to balance the different levels' relative feature densities.
      lev.vCorners.clear();
      lev.vCandidates.clear();
      lev.vMaxCorners.clear();

      if (!nCam){
          if (imnum == 0)// for landing pad ref img
          {
              if(i == 0)
                  fast_corner_detect_10(lev.im, lev.vCorners, 20);
              if(i == 1)
                  fast_corner_detect_10(lev.im, lev.vCorners, 20);
              if(i == 2)
                  fast_corner_detect_10(lev.im, lev.vCorners, 14);
              if(i == 3)
                  fast_corner_detect_10(lev.im, lev.vCorners, 12);
          }
          else
          {
              if(i == 0)
                  fast_corner_detect_10(lev.im, lev.vCorners, 20);//16
              if(i == 1)
                  fast_corner_detect_10(lev.im, lev.vCorners, 18);//14
              if(i == 2)
                  fast_corner_detect_10(lev.im, lev.vCorners, 14);//12
              if(i == 3)
                  fast_corner_detect_10(lev.im, lev.vCorners, 14);
          }
      }
      else
      {
          if(i == 0)
              fast_corner_detect_10(lev.im, lev.vCorners, 22);//10
          if(i == 1)
              fast_corner_detect_10(lev.im, lev.vCorners, 20);//12
          if(i == 2)
              fast_corner_detect_10(lev.im, lev.vCorners, 14);
          if(i == 3)
              fast_corner_detect_10(lev.im, lev.vCorners, 12);
      }

//      std::cout << "lev size: " << lev.vCorners.size() << std::endl;
      // On Pixhawk, block the parts of image from the landing gear
//      if (imnum){// ignore this when loading ref landing pad
//          for (int j = 0; j < lev.vCorners.size(); j ++){
//              if ( (lev.vCorners[j].x < x1/(1 << i)) || (lev.vCorners[j].x > x2/(1 << i))){
//                  lev.vCorners.erase(lev.vCorners.begin() + j);
//                  j--;
//              }
//          }
//      }
//      std::cout << "lev size: " << lev.vCorners.size() << std::endl;


      lev.vCornersDepth.resize(lev.vCorners.size(),0.);
      
      createRowLookupTable(i);
    };
  imnum = 1;
}

void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  
  // For each level...
  for(int l=0; l<LEVELS; l++)
    {
      Level &lev = aLevels[l];
      // .. find those FAST corners which are maximal..
      fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
      lev.vMaxCornersDepth.resize(lev.vMaxCorners.size(),0.0);
      // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
      // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
      // to make new map points out of.
      unsigned int ind = 0;
      for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++, ind++)
	{
	  if(!lev.im.in_image_with_border(*i, 10))
	    continue;
	  double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
	  if(dSTScore > *gvdCandidateMinSTScore)
	    {
	      Candidate c;
	      c.irLevelPos = *i;
              c.dSTScore = dSTScore;
              c.dDepth = 0.0;
              lev.vCandidates.push_back(c);
	    }
	}
    }
  
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  SBI = SmallBlurryImage(*this);  
  // Relocaliser also wants the jacobians..
  SBI.MakeJacs();
  bComplete = true;

  // for landing pad detection
  islandingpadDetected = false;
  mPadCorners.clear();
  mMapPointsInPad.clear();
}

void KeyFrame::MakeKeyFrame(BasicImage<byte> &im,CVD::BasicImage<uint16_t> &depth, CameraModel* cam)
{
    MakeKeyFrame_Lite(im);

    for (int l = 0; l < LEVELS; l++) {
        Level& lev = aLevels[l];
        lev.vCornersDepth.resize(lev.vCorners.size());
        for (unsigned int i = 0; i < lev.vCorners.size(); i++) {
            ImageRef irLev = lev.vCorners[i];
            ImageRef irL0 = LevelZeroPosIR(irLev,l);
            double d = depth[irL0]/1000.0;
            d = isnan(d) ? 0.0 : d;
            lev.vCornersDepth[i] = d;
        }
    }

    // Fills the rest of the keyframe structure needed by the mapmaker:
    // FAST nonmax suppression, generation of the list of candidates for further map points,
    // creation of the relocaliser's SmallBlurryImage.
    static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);

    // For each level...
    for(int l=0; l<LEVELS; l++)
      {
        Level &lev = aLevels[l];
        // .. find those FAST corners which are maximal..
        fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
        // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
        // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
        // to make new map points out of.
        unsigned int ind = 0;
        lev.vMaxCornersDepth.resize(lev.vMaxCorners.size());
        for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++, ind++)
          {

              ImageRef irL0 = LevelZeroPosIR(*i,l);
              double d = depth[irL0]/1000.0;
              d = isnan(d) ? 0.0 : d;
              lev.vMaxCornersDepth[ind] = d;

              if(!lev.im.in_image_with_border(*i, 10))
                  continue;
              double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
              if(dSTScore > *gvdCandidateMinSTScore)
              {
                  Candidate c;
                  c.irLevelPos = *i;
                  c.dSTScore = dSTScore;
                  c.dDepth = d;
                  lev.vCandidates.push_back(c);
              }
          }

//        std::cout << __PRETTY_FUNCTION__ << " level: " << l << ", candidates: " << lev.vCandidates.size() << std::endl;
      };

    // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
    SBI = SmallBlurryImage(*this);
    // Relocaliser also wants the jacobians..
    SBI.MakeJacs();
    bComplete = true;
}

// Initializes a keyframe with data from sparse stereo matching
void KeyFrame::MakeKeyFrame(CVD::BasicImage<CVD::byte> &im, const sensor_msgs::PointCloud& points, CameraModel* cam) {
	// Find the channels for image coordinates
	unsigned int uChannel = 0, vChannel = 1, octChannel = 2;
	for(unsigned int i=0; i<points.channels.size(); i++)
		if(points.channels[i].name == "u")
			uChannel = i;
		else if(points.channels[i].name == "v")
			vChannel = i;
		else if(points.channels[i].name == "octave")
			octChannel = i;
	
	static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);

    // Create pyramid images and reset everything
	for(int l=0; l<LEVELS; l++) {
		Level &lev = aLevels[l];
		if(l!=0) {
			// .. make a half-size image from the previous level..
			lev.im.resize(aLevels[l-1].im.size() / 2);
			halfSample(aLevels[l-1].im, lev.im);
		} else {
			lev.im.resize(im.size());
			copy(im, lev.im);
		}

		// Reset frame
		lev.vCorners.clear();
		lev.vCornersDepth.clear();
		lev.vCandidates.clear();
		
		if(l == 0) {
			// In this case we already know the number of features
			lev.vCorners.reserve(points.points.size());
			lev.vCornersDepth.reserve(points.points.size());
			lev.vCandidates.reserve(points.points.size());
		}
	}
				
	// Add feature points to pyramid
	for(unsigned int i=0; i<points.points.size(); i++) {
		for(int l=0, factor=1; l<= points.channels[octChannel].values[i] && l<LEVELS; l++, factor*=2) {

			Level &lev = aLevels[l];
			lev.vCorners.push_back(ImageRef(int(points.channels[vChannel].values[i])/factor,
				int(points.channels[uChannel].values[i])/factor));
			lev.vCornersDepth.push_back(points.points[i].z);
			
			// Determine if this is a good mapping candidate
			if(!lev.im.in_image_with_border(lev.vCorners.back(), 10))
				continue;
				
			/*double dSTScore = FindShiTomasiScoreAtPoint(aLevels[0].im, 3, aLevels[0].vCorners.back());
			if(dSTScore > *gvdCandidateMinSTScore)*/ {
				Candidate c;
				c.irLevelPos = lev.vCorners.back();
				c.dSTScore = *gvdCandidateMinSTScore+1;//dSTScore;
				c.dDepth = points.points[i].z;
				lev.vCandidates.push_back(c);
			}
		}
	}
	
	// Do final things
	for(int l=0; l<LEVELS; l++) {
		Level &lev = aLevels[l];
		// There is no difference between corners and maxCorners for stereo data
		lev.vMaxCorners = lev.vCorners;
		lev.vMaxCornersDepth = lev.vCornersDepth;
		
		createRowLookupTable(l);
	}
	
	// Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
	SBI = SmallBlurryImage(*this);
	// Relocaliser also wants the jacobians..
	SBI.MakeJacs();
	bComplete = true;
}

void KeyFrame::createRowLookupTable(int level) {
	// Generate row look-up-table for the FAST corner points: this speeds up 
	// finding close-by corner points later on.
	unsigned int v=0;
	aLevels[level].vCornerRowLUT.clear();
	aLevels[level].vCornerRowLUT.reserve(aLevels[level].im.size().y);
	for(int y=0; y<aLevels[level].im.size().y; y++) {
		while(v < aLevels[level].vCorners.size() && y > aLevels[level].vCorners[v].y)
			v++;
		aLevels[level].vCornerRowLUT.push_back(v);
	}
}

void KeyFrame::finalizeKeyframeBackend()
{
    if(finalized || !mMeasurements.size())
        return;
    /// TODO: descript points on the correct level!

    mapPoints.clear();
    for(const_meas_it it = mMeasurements.begin(); it != mMeasurements.end(); it++) {
        boost::shared_ptr<MapPoint> point = it->first;
        if (point->pPatchSourceKF.lock() //&& source kf not removed
//                !point->sourceKfIDtransfered && // avoid re-send
//                point->pPatchSourceKF.lock()->id == id
                )
        {// point belongs to this kf

            // relative pose of the map points to this kf! (not neccecerrily the source kf!):
            if (point->pPatchSourceKF.lock()->id != id)
                point->v3RelativePos = se3CfromW*point->v3WorldPos;
            point->irCenterZero = LevelZeroPosIR(point->irCenter,point->nSourceLevel);

            mapPoints.push_back(point);
        }
    }
    cout << "mapPoints: " << mapPoints.size() << endl;

    boost::scoped_ptr<cv::DescriptorExtractor> extractor(new cv::BriefDescriptorExtractor(32));

    std::vector<std::vector<cv::KeyPoint> >mpKpts(LEVELS);
    for (uint i = 0; i < mapPoints.size(); i++) {
        int l = mapPoints[i]->nSourceLevel;
        cv::KeyPoint kp(cv::Point2f(mapPoints[i]->irCenterZero[0], mapPoints[i]->irCenterZero[1]),
                        (1 << l)*10, // TODO: feature size to be adjusted
                        -1, 0,
                        l);

        kp.class_id = i; // store index of original map point
        mpKpts[l].push_back(kp);
    }

    // compute descriptors of map points and all corners(for loop detection)
    Level &lev = aLevels[0];
    ImageRef irsize = lev.im.size();

    // Make sure image is actually copied:
    cv::Mat cv_im = cv::Mat(irsize[1], irsize[0], CV_8UC1,
                        (void*) lev.im.data(), lev.im.row_stride()).clone();
    std::vector<boost::shared_ptr<MapPoint> > mpNew;
    mpKeypoints.clear();
    keypoints.clear();
    mpDescriptors.release();
    kpDescriptors.release();
    for (uint l = 0; l < 2; l++) {
        cv::Mat levDesc;
        // Warning: this modifies kpts (deletes keypoints for which it cannot compute a descriptor)
        extractor->compute(cv_im, mpKpts[l], levDesc);

        assert(mpKpts[l].size() == levDesc.rows);

        mpDescriptors.push_back(levDesc);

        // Keep only map points with descriptors:
        for (int i = 0; i < mpKpts[l].size(); i++) {
            mpKeypoints.push_back(mpKpts[l][i]);
            mpNew.push_back(mapPoints[mpKpts[l][i].class_id]);
        }
        assert(mpDescriptors.rows == mpKeypoints.size());

        // for all corners
        Level &lev1 = aLevels[l];
        int scaleFactor = (1 << l);
        std::vector<cv::KeyPoint> kpts;
        kpts.resize(lev1.vMaxCorners.size());

        for (uint k = 0; k < kpts.size(); k++) {
            kpts[k].pt.x = lev1.vMaxCorners[k].x*scaleFactor;
            kpts[k].pt.y = lev1.vMaxCorners[k].y*scaleFactor;
            kpts[k].octave = l;
            kpts[k].angle = -1;
            kpts[k].size = (1 << l)*10; // setting this is required for DescriptorExtractors to work.

            // Abuse response field, but we have to keep this keypoint's depth somewhere
            kpts[k].response = lev1.vMaxCornersDepth[k];
        }
        cout << "IN the current kf, mpKpts, keypoints: "<<
                mpKpts[l].size() << ", " << kpts.size() << endl;

        cv::Mat levkpDesc;
        // Warning: this modifies kpts (deletes keypoints for which it cannot compute a descriptor)
        extractor->compute(cv_im, kpts, levkpDesc);

        assert(levkpDesc.rows == kpts.size());

        // concatenate all levels
        kpDescriptors.push_back(levkpDesc);
        keypoints.insert( keypoints.end(), kpts.begin(), kpts.end() );
    }
    mapPoints = mpNew;

    cout << "mapPoints: " << mapPoints.size() << endl;
    cout << "IN the current kf, all describ. all keypoints: "
         << kpDescriptors.rows << ", " << keypoints.size() << endl;

    // extract depth again
    kpDepth.resize(keypoints.size());
    for (unsigned int i = 0; i < keypoints.size(); i++) {
        kpDepth[i] = keypoints[i].response;
    }

    finalized = true;
}

void KeyFrame::finalizeKeyframekpts()
{
    static boost::scoped_ptr<cv::DescriptorExtractor> extractor(new cv::BriefDescriptorExtractor(32));

    // compute descriptors of all corners
    // Make sure image is actually copied:
    Level &lev = aLevels[0];
    ImageRef irsize = lev.im.size();
    cv::Mat cv_im = cv::Mat(irsize[1], irsize[0], CV_8UC1,
                        (void*) lev.im.data(), lev.im.row_stride()).clone();
    keypoints.clear();
    kpDescriptors.release();
    for (uint l = 0; l < 2; l++) {
        Level &lev1 = aLevels[l];

        // for all corners
        int scaleFactor = (1 << l);
        std::vector<cv::KeyPoint> kpts;
        kpts.resize(lev1.vMaxCorners.size());

        for (uint k = 0; k < kpts.size(); k++) {
            kpts[k].pt.x = lev1.vMaxCorners[k].x*scaleFactor;
            kpts[k].pt.y = lev1.vMaxCorners[k].y*scaleFactor;
            kpts[k].octave = l;
            kpts[k].angle = -1;
            kpts[k].size = (1 << l)*10; // setting this is required for DescriptorExtractors to work.

            // Abuse response field, but we have to keep this keypoint's depth somewhere
            kpts[k].response = lev1.vMaxCornersDepth[k];
        }

        cout << "in the current kf, keypoints: "<< kpts.size() << endl;

        cv::Mat levkpDesc;
        // Warning: this modifies kpts (deletes keypoints for which it cannot compute a descriptor)
        extractor->compute(cv_im, kpts, levkpDesc);

        assert(levkpDesc.rows == kpts.size());

        // concatenate all levels
        kpDescriptors.push_back(levkpDesc);
        keypoints.insert( keypoints.end(), kpts.begin(), kpts.end() );
    }
    cout << "in the current kf for reloc., all kpdesc. keypoints: " << kpDescriptors.rows << ", " << keypoints.size() << endl;

    // extract depth again
    kpDepth.resize(keypoints.size());
    for (unsigned int i = 0; i < keypoints.size(); i++) {
        kpDepth[i] = keypoints[i].response;
    }
}

//void KeyFrame::createRowLookupTable_sec(int level) {
//    // Generate row look-up-table for the FAST corner points: this speeds up
//    // finding close-by corner points later on.
//    unsigned int v=0;
//    aLevels_sec[level].vCornerRowLUT.clear();
//    aLevels_sec[level].vCornerRowLUT.reserve(aLevels_sec[level].im.size().y);
//    for(int y=0; y<aLevels_sec[level].im.size().y; y++) {
//        while(v < aLevels_sec[level].vCorners.size() && y > aLevels_sec[level].vCorners[v].y)
//            v++;
//        aLevels_sec[level].vCornerRowLUT.push_back(v);
//    }
//}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
Level& Level::operator=(const Level &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.resize(rhs.im.size());
  copy(rhs.im, im);
  
  vCorners = rhs.vCorners;
  vCornersDepth = rhs.vCornersDepth;
  vMaxCorners = rhs.vMaxCorners;
  vMaxCornersDepth = rhs.vCornersDepth;
  vCornerRowLUT = rhs.vCornerRowLUT;
  vCandidates = rhs.vCandidates;

  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
      {
	if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
	else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
	else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
	else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 0.7);
	else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
      }
  }
};

static LevelHelpersFiller foo;
