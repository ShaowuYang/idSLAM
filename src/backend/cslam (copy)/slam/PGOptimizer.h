#ifndef _CSLAM_PG_OPTIMIZER_H_
#define _CSLAM_PG_OPTIMIZER_H_

#include <boost/scoped_ptr.hpp>
#include <g2o/core/graph_optimizer_sparse.h>
//#include <slam/Keyframe.h>

namespace cslam {

class PGOptimizer {
public:
//    PGOptimizer(const Sophus::SE3d& bodyTcam);
    PGOptimizer();

    void reset();

    void addKeyframe(const Keyframe& kf);
    void addEdge(const Edge& e, bool useScenDepth=true, double scenDepth=1.0);
    void addLoopEdgeSim3(const Edge& e, double scale);
    void optimize();

    // define a subgraph to be optimised when no loop is detected
    // @vnum: maximul number of verteces in the subgraph
    bool defineSubGraph(g2o::HyperGraph::VertexSet& v, const std::set<int> vids);
    void optimize_portion(const std::set<int> vids);// optimise a portion of the graph
    void setScenDepth(double maxdepth, double mindepth, double mininfo = 0.5){
       maxScenDepth = maxdepth;
       minScenDepth = mindepth;
       minInform = mininfo;
    }

    void applyResult(std::vector<boost::shared_ptr<Keyframe> > keyframes);
protected:
    int nIters_;
    Sophus::SE3d bodyTcam_;

    bool firstRun_;
    g2o::HyperGraph::VertexSet verticesAdded;
    g2o::HyperGraph::EdgeSet edgesAdded;
    g2o::HyperGraph::VertexSet vportion;
    boost::scoped_ptr<g2o::SparseOptimizer> optimizer_;

    int lastPV_;
    std::map<int, int> kf2pv_, pv2kf_;
    unsigned int idVfixed;// the fixed vertex in portion graph optimisation, as the origin

    double maxScenDepth;// expected max and min scene depth
    double minScenDepth;
    double minInform; // minimal information value for the max scene depth e.g. 0.5~1.0
};

}

#endif /* _CSLAM_PG_OPTIMIZER_H_ */
