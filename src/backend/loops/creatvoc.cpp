/**
 * File: Demo.cpp
 * Date: November 2011
<<<<<<< HEAD
 * Author: Dorian Galvez-Lopez Modified by Yang with boost filesystem
=======
 * Author: Dorian Galvez-Lopez
>>>>>>> 8243fcbc2be49933bf15148437088c9be9352b56
 * Description: demo application of DBoW2
 */

#include <iostream>
#include <vector>
// boost filesystem
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>//v3

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/features2d.hpp>

// DBoW2
#include <DBoW2/DBoW2.h> // defines Surf64Vocabulary and Surf64Database

#include <DBoW2/DUtils.h>
#include <DBoW2/DVision.h>

using namespace cv;
using namespace DBoW2;
using namespace DUtils;
using namespace std;
using namespace boost::filesystem;//filesystem3

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<FBrief::TDescriptor> > &features);
//void changeStructure(const vector<float> &plain, vector<FBrief::TDescriptor> &out, int L);
void testVocCreation(const vector<vector<FBrief::TDescriptor> > &features);
void testDatabase(const vector<vector<FBrief::TDescriptor> > &features);
void creatVoc(const vector<vector<FBrief::TDescriptor> > &features);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 11;
unsigned int nImages;


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
    nImages = 0;
  vector<vector<FBrief::TDescriptor> > features;
  loadFeatures(features);

  if (features.size())
    creatVoc(features);

  wait();
  return 0;

//  testDatabase(features);

}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<FBrief::TDescriptor> > &features)
{
  features.clear();
  features.reserve(NIMAGES);

  DVision::BRIEF brief(256, 48);
  Ptr<FeatureDetector> detector = new PyramidAdaptedFeatureDetector(new FastFeatureDetector(),4);

  cout << "Extracting BRIEF features of images in " << endl;

  path pimgs("images/vocimages");
  if (is_directory(pimgs)){
      cout << pimgs.c_str() <<"..." << endl;

      typedef vector<path> vec;             // store paths,
              vec vpath;                                // so we can sort them later
      copy(directory_iterator(pimgs), directory_iterator(), // directory_iterator::value_type
           back_inserter(vpath));
      sort(vpath.begin(), vpath.end());
      nImages = vpath.size();

      for (vec::const_iterator it (vpath.begin()); it != vpath.end(); ++it)
      {
        stringstream ss;
        ss << (*it).c_str();

        cv::Mat image = cv::imread(ss.str(), 0);
        cv::Mat mask;
        vector<cv::KeyPoint> keypoints;
        vector<FBrief::TDescriptor> descriptors;

        detector->detect(image, keypoints);
        brief(image, keypoints, descriptors);

        cout << "image: " << ss.str().c_str() << " features: " << keypoints.size() << endl;

        features.push_back(descriptors);
        cv::imshow("image", image);
        cv::waitKey(100);
    //    changeStructure(descriptors, features.back(), brief.descriptorSize());
      }
  }
}

// ----------------------------------------------------------------------------

//void changeStructure(const vector<float> &plain, vector<vector<float> > &out,
//  int L)
//{
//  out.resize(plain.size() / L);

//  unsigned int j = 0;
//  for(unsigned int i = 0; i < plain.size(); i += L, ++j)
//  {
//    out[j].resize(L);
//    std::copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
//  }
//}

// ----------------------------------------------------------------------------
void creatVoc(const vector<vector<FBrief::TDescriptor> > &features)
{
  // branching factor and depth levels
  const int k = 10;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  BriefVocabulary voc(k, L, weight, score);

  cout << "Creating " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;
  for(int i = 0; i < nImages; i++)
  {
    voc.transform(features[i], v1);
    for(int j = 0; j < nImages; j++)
    {
      voc.transform(features[j], v2);

      double score = voc.score(v1, v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("data/indoor_voc.yml.gz");
  cout << "Done" << endl;
}

void testVocCreation(const vector<vector<FBrief::TDescriptor> > &features)
{
  // branching factor and depth levels 
  const int k = 9;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  BriefVocabulary voc(k, L, weight, score);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;
  for(int i = 0; i < NIMAGES; i++)
  {
    voc.transform(features[i], v1);
    for(int j = 0; j < NIMAGES; j++)
    {
      voc.transform(features[j], v2);
      
      double score = voc.score(v1, v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<FBrief::TDescriptor> > &features)
{
  cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  BriefVocabulary voc("small_voc.yml.gz");

  BriefDatabase db(voc, false); // false = do not use direct index.
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < NIMAGES; i++)
  {
    db.add(features[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << db << endl;

  // and query the database
  cout << "Querying the database: " << endl;

  QueryResults ret;
  for(int i = 0; i < NIMAGES; i++)
  {
    db.query(features[i], ret, 4);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    cout << "Searching for Image " << i << ". " << ret << endl;
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  cout << "... done!" << endl;

  // once saved, we can load it again  
  cout << "Retrieving database once again..." << endl;
  BriefDatabase db2("small_db.yml.gz");
  cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------


