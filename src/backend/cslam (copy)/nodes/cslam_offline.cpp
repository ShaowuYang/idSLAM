#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "CSLAMNode.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cslam_offline");

    if (argc != 2) {
        cout << "usage: " << argv[0] << " filename.bag" << endl;
        return EXIT_FAILURE;
    }

    ros::NodeHandle n;
    cslam::CSLAMNode node(n);

    std::string bagfile = argv[1];

    cout << "trying to open bag file: " << bagfile << endl;
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    std::string kf_topic    = "/ptam/keyframes";
    std::string edges_topic = "/ptam/edges";

    std::vector<std::string> topics;
    topics.push_back(kf_topic);
    topics.push_back(edges_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Process one message after the other, disregarding time,
    // by manually calling the corresponding callbacks.
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == kf_topic) {
            cs_ptam_msgs::Keyframe::ConstPtr kf = m.instantiate<cs_ptam_msgs::Keyframe>();
            if (kf != NULL)
                node.keyframeCb(kf);
        }

        if (m.getTopic() == edges_topic) {
            cs_ptam_msgs::Edges::ConstPtr edges = m.instantiate<cs_ptam_msgs::Edges>();
            if (edges != NULL)
                node.edgesCb(edges);
        }

        for (int i = 0; i < 20; i++) ros::spinOnce();
    }
    bag.close();

    return EXIT_SUCCESS;
}

