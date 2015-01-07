
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <DBoW2/DBoW2.h>

using namespace boost;
# if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION == 3
using namespace boost::filesystem3;
# else
using namespace boost::filesystem;
# endif

using namespace cv;
using namespace std;



int main(int argc, char **argv)
{
    if (argc != 4) {
        cout << "usage: " << argv[0] << " k l directory" << endl;
        return 1;
    }

    int k = atoi(argv[1]);
    int l = atoi(argv[2]);
    string dir = argv[3];
    path path(dir);
    if (!exists(path)) {
        cout << "no such directory: " << dir << endl;
        return 1;
    }

    vector<string> filenames;

    directory_iterator it_end;
    for (directory_iterator it(path); it != it_end; it++) {
        if (is_directory(it->status())) {
            continue;
        }

        filenames.push_back(it->path().string());
    }

    vector<vector<DBoW2::FBrief::TDescriptor> > dbow_desc;
    cout << "reading images and extracting features..." << endl;
    boost::scoped_ptr<FeatureDetector> detector(new PyramidAdaptedFeatureDetector(new FastFeatureDetector(8), 4));
    boost::scoped_ptr<DescriptorExtractor> extractor(new BriefDescriptorExtractor(32));

    boost::mutex combine_mutex;
    #pragma omp parallel for
    for (unsigned int i = 0; i < filenames.size(); i++) {
        const string& filename = filenames[i];
        cv::Mat img =  cv::imread(filename, cv::IMREAD_GRAYSCALE);

        vector<KeyPoint> keypoints;
        detector->detect(img, keypoints);
        cv::Mat descriptors;
        extractor->compute(img, keypoints, descriptors);

        vector<DBoW2::FBrief::TDescriptor> this_desc(descriptors.rows);
        for (int i = 0; i < descriptors.rows; i++) {
            DBoW2::FBrief::fromMat8UC(this_desc[i], descriptors.row(i));
        }

        boost::lock_guard<boost::mutex> lock(combine_mutex);
        dbow_desc.push_back(this_desc);
    }

    cout << "... done reading images and extracting features!" << endl;
    cout << "Creating vocabulary..." << endl;
    BriefVocabulary voc;
    voc.create(dbow_desc, k, l);
    cout << "done creating vocabulary!" << endl;

    cout << "Saving vocabulary..." << endl;
    voc.save("vocabulary.voc");
    cout << "...done saving vocabulary." << endl;

    return 0;
}

