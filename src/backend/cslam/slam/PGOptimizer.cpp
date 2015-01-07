#include "PGOptimizer.h"

#include <boost/foreach.hpp>
#include <queue>

#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <g2o/types/slam3d/edge_se3_quat.h>
//#include <g2o_types/anchored_points.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <cs_geometry/Conversions.h>

#include "PlaneEdge.h"

using namespace cslam;
using namespace std;

#define SPACE_SIM3 0
#define maxPointDepth 5.0
#define minPointDepth 1.0

PGOptimizer::PGOptimizer()
{
    reset();
}

void PGOptimizer::reset()
{
    firstRun_ = true;

    bodyTcam_ = Sophus::SE3d();
    nIters_  = 5;
    setScenDepth(maxPointDepth, minPointDepth, 0.5);

    // Setup g2o:
    optimizer_.reset(new g2o::SparseOptimizer());

    // TODO: try sim3 from g2o or ScaViSLAM
#if SPACE_SIM3
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<7, 3> >  SlamBlockSolver;
    g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>* linearSolver =
            new g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *solver = new SlamBlockSolver(optimizer_.get(), linearSolver);

    optimizer_->setSolver(solver);
    optimizer_->setVerbose(false);

    g2o::VertexSim3Expmap *v = new g2o::VertexSim3Expmap();
    v->setId(0);
    v->setToOrigin();
    v->setFixed(true);
    v->_fix_scale = true;

    bool ok = optimizer_->addVertex(v);
    verticesAdded.insert(v);
#else
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
    g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>* linearSolver =
            new g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *solver = new SlamBlockSolver(optimizer_.get(), linearSolver);

    optimizer_->setSolver(solver);
    optimizer_->setVerbose(false);

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(0);
    // Proper initialization required:

    v->setEstimate(g2o::SE3Quat());
    v->setFixed(true);
    bool ok = optimizer_->addVertex(v);
    verticesAdded.insert(v);
#endif
    assert(ok);

    lastPV_ = 0; // reference vertex
    kf2pv_.clear();
    pv2kf_.clear();

    kf2pv_[-1] = 0;
    pv2kf_[0] = -1;

    idVfixed = 0;
}

void PGOptimizer::addKeyframe(const ptam::KeyFrame& kf)
{
    // Determine new pose graph vertex id:
    int thisPV = ++lastPV_;

    kf2pv_[kf.id] = thisPV;
    pv2kf_[thisPV] = kf.id;

#if SPACE_SIM3
    g2o::VertexSim3Expmap *v = new g2o::VertexSim3Expmap();
    v->setId(thisPV);
    // the SIM3 library use inverse representation as we espected
    Sophus::SE3d posectw = cs_geom::toSophusSE3(kf.se3CfromW);
    v->setEstimate(g2o::Sim3(posectw.rotationMatrix(), posectw.translation(), 1.));
    if (thisPV == 1){
        v->setFixed(true);
        v->_fix_scale = true;
    }

    bool ok = optimizer_->addVertex(v);
    verticesAdded.insert(v);
#else
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(thisPV);
    v->setEstimate(g2o::SE3Quat(cs_geom::toSophusSE3(kf.se3CfromW.inverse()).rotationMatrix(), cs_geom::toSophusSE3(kf.se3CfromW.inverse()).translation()));

    bool ok = optimizer_->addVertex(v);
    verticesAdded.insert(v);
#endif
    assert(ok);

    if (thisPV == 1) { // first "real" vertex
        addEdge(ptam::Edge(pv2kf_.at(0), kf.id, ptam::EDGE_INIT, cs_geom::toSophusSE3(kf.se3CfromW.inverse())),
                kf.dSceneDepthMean);
    }
}

void PGOptimizer::addEdge(const ptam::Edge& e, double scenDepth, bool useScenDepth)
{
#if SPACE_SIM3
    g2o::EdgeSim3 *ge = new g2o::EdgeSim3();
    g2o::OptimizableGraph::Vertex *vA = optimizer_->vertex(kf2pv_.at(e.idA));
    g2o::OptimizableGraph::Vertex *vB = optimizer_->vertex(kf2pv_.at(e.idB));
    assert(vA);
    assert(vB);
    ge->vertices()[0] = vA;
    ge->vertices()[1] = vB;
    // the SIM3 library use inverse representation as we espected
    Sophus::SE3d Tbta = e.aTb.inverse();
    g2o::Sim3 Sbta(Tbta.rotationMatrix(), Tbta.translation(), 1.);
    ge->setMeasurement(Sbta);
    ge->setInverseMeasurement(Sbta.inverse());

    Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    I(0,0) = 0.01;// r t s
    I(1,1) = 0.01;
    I(2,2) = 0.01;
    I(6,6) = 0.01;// TODO: set differently according to edge type
#else
    g2o::EdgeSE3 *ge = new g2o::EdgeSE3();

    g2o::OptimizableGraph::Vertex *vA = optimizer_->vertex(kf2pv_.at(e.idA));
    g2o::OptimizableGraph::Vertex *vB = optimizer_->vertex(kf2pv_.at(e.idB));
    assert(vA);
    assert(vB);
    ge->vertices()[0] = vA;
    ge->vertices()[1] = vB;
    ge->setMeasurement(g2o::SE3Quat(e.aTb.rotationMatrix(), e.aTb.translation()));

    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    I(0,0) = 0.01;
    I(1,1) = 0.01;
    I(2,2) = 0.01;

    if (useScenDepth){
        if (scenDepth > maxPointDepth)
            scenDepth = maxPointDepth;
        I(3,3) = minInform + abs(maxScenDepth - scenDepth) * abs(1.0 - minInform) / (maxScenDepth - minScenDepth);
        I(4,4) = I(3,3);
        I(5,5) = I(3,3);
    }
#endif

    double weight = 1.0;
    switch (e.type) {
    case ptam::EDGE_MOTIONMODEL:
        weight = 0.001;
        break;
    case ptam::EDGE_LOOP:
        weight = 1.0;
        break;
    case ptam::EDGE_PTAM:
        weight = 1.0;
        break;
    case ptam::EDGE_INIT:
        weight = 0.001;
        break;
    default:
        assert(false);
        break;
    }

    ge->setInformation(I*weight);

    bool ok = optimizer_->addEdge(ge);
    edgesAdded.insert(ge);
    assert(ok);
}

void PGOptimizer::addLoopEdgeSim3(const ptam::Edge& e, double scale)
{
    assert(SPACE_SIM3);

    g2o::EdgeSim3 *ge = new g2o::EdgeSim3();
    g2o::OptimizableGraph::Vertex *vA = optimizer_->vertex(kf2pv_.at(e.idA));
    g2o::OptimizableGraph::Vertex *vB = optimizer_->vertex(kf2pv_.at(e.idB));
    assert(vA);
    assert(vB);
    // the SIM3 library use inverse representation as we espected
    Sophus::SE3d Tbta = e.aTb.inverse();
    ge->setMeasurement(g2o::Sim3(Tbta.rotationMatrix(), Tbta.translation(), 1.));

    Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    I(0,0) = 0.01;
    I(1,1) = 0.01;
    I(2,2) = 0.01;
    I(3,3) = 0.01;

    double weight = 1.0;
    switch (e.type) {
    case ptam::EDGE_MOTIONMODEL:
        weight = 0.001;
        break;
    case ptam::EDGE_LOOP:
        weight = 1.0;
        break;
    case ptam::EDGE_PTAM:
        weight = 1.0;
        break;
    case ptam::EDGE_INIT:
        weight = 0.001;
        break;
    default:
        assert(false);
        break;
    }

    ge->setInformation(I*weight);

    bool ok = optimizer_->addEdge(ge);
    edgesAdded.insert(ge);
    assert(ok);
}

void PGOptimizer::optimize()
{
    // if use the double window spirit, we can ini a portion of the graph when we want to do local pgo
    if (firstRun_) {
        bool initOK = optimizer_->initializeOptimization();
        assert(initOK);
    } else { // this is not the first run: update instead of initializing
        bool updateOK = optimizer_->updateInitialization(verticesAdded, edgesAdded);
        assert(updateOK);
    }
    optimizer_->optimize(nIters_, !firstRun_);
    verticesAdded.clear();
    edgesAdded.clear();
    firstRun_ = false;
}

void PGOptimizer::optimize_portion(const set<int> vids)
{
    bool optportion = false;
    // if use the double window spirit, we can ini a portion of the graph when we want to do local pgo
    if (firstRun_) {
        bool initOK = optimizer_->initializeOptimization();
        assert(initOK);
    } else {
        // this is not the first run: define a portion of the graph
        if (!defineSubGraph(vportion, vids))
            return;

#if SPACE_SIM3
        // set a fixed vertex
        g2o::VertexSim3Expmap *vfix = (g2o::VertexSim3Expmap*) optimizer_->vertex(kf2pv_.at(idVfixed));
        vfix->setFixed(true);
        vfix->_fix_scale = true;
#else
        // set a fixed vertex
        g2o::VertexSE3 *vfix = (g2o::VertexSE3*) optimizer_->vertex(kf2pv_.at(idVfixed));
        vfix->setFixed(true);
#endif

        // initialize the defined portion
        bool updateOK = optimizer_->initializeOptimization(vportion);
        assert(updateOK);
        optportion = true;
    }
    cout<< "doing optimizer_->optimize..." << endl;
    optimizer_->optimize(nIters_);//, !firstRun_);
    cout<< "done optimizer_->optimize..." << endl;

    verticesAdded.clear();
    edgesAdded.clear();
    vportion.clear();
    firstRun_ = false;
}

bool PGOptimizer::defineSubGraph(g2o::HyperGraph::VertexSet& v, const set<int> vids)
{
    idVfixed = kf2pv_.size();
    for (set<int>::iterator it = vids.begin(); it != vids.end(); it ++){
        assert(*it < kf2pv_.size()-1);

#if SPACE_SIM3
        g2o::VertexSim3Expmap* vi = (g2o::VertexSim3Expmap*) optimizer_->vertex(kf2pv_.at(*it));
#else
        g2o::VertexSE3* vi = (g2o::VertexSE3*) optimizer_->vertex(kf2pv_.at(*it));
#endif
        v.insert(vi);

        if (vi->id() < idVfixed && vi->id() >= 0)
            idVfixed = vi->id();
    }
    if (v.size())
        return true;
    return false;
}

void PGOptimizer::applyResult(std::vector<boost::shared_ptr<ptam::KeyFrame> > keyframes, std::vector<Sophus::SE3d>& keyframedPoses)
{
    keyframedPoses.resize(keyframes.size());
    for (int i = idVfixed; i < keyframes.size(); i++) {
        int id = keyframes[i]->id;
//        boost::shared_ptr<ptam::KeyFrame> kf = keyframes[i];
//        keyframedPoses[i] = cs_geom::toSophusSE3(kf->se3CfromW);

#if SPACE_SIM3
        g2o::VertexSim3Expmap* v = (g2o::VertexSim3Expmap*) optimizer_->vertex(kf2pv_.at(id));
        const g2o::Sim3 est = v->estimate();
        // the SIM3 library use inverse representation as we espected
        Sophus::SE3d Tctw;
        Tctw.setQuaternion(est.rotation());
        Tctw.translation() = est.translation()*est.scale();
        kf->se3CfromW = cs_geom::toToonSE3(Tctw);
        // TODO: and update the map points
#else
        g2o::VertexSE3* v = (g2o::VertexSE3*) optimizer_->vertex(kf2pv_.at(id));
        const g2o::SE3Quat est = v->estimate();
        Sophus::SE3d poseNew;
        poseNew.setQuaternion(est.rotation());
        poseNew.translation() = est.translation();
        keyframedPoses[i] = poseNew.inverse(); // pose cfw
//        kf->se3CfromW = cs_geom::toToonSE3(poseNew).inverse();
#endif
    }
}
