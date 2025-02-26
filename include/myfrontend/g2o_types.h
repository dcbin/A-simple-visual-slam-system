#ifndef G2O_TYPES_H
#define G2O_TYPES_H
#include "camera.h"
#include "common_include.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace myfrontend {

using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
using optimizer = g2o::OptimizationAlgorithmLevenberg;

class EdgeProjectXYZ2UV: 
    public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                               g2o::VertexPointXYZ,
                               g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& out){}
    Camera::Ptr camera_;
};
} // namespace myfrontend
#endif