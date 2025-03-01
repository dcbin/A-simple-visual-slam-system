#ifndef G2O_TYPES_H
#define G2O_TYPES_H
#include "camera.h"
#include "common_include.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace myfrontend {

using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
using optimizer = g2o::OptimizationAlgorithmLevenberg;

/**
 * @brief 优化相机位姿和3D点坐标
 */
class EdgeProjectXYZ2UV: 
    public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                               g2o::VertexPointXYZ,
                               g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream& in) {return false;}
    virtual bool write(std::ostream& out) const {return false;}
    Camera *camera_ = nullptr;
};

class EdgeProjectPoseOnly:
    public g2o::BaseUnaryEdge<2, Eigen::Vector2d,
                              g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectPoseOnly() = default;
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream& in) {return false;}
    virtual bool write(std::ostream& out) const {return false;}
    Camera *camera_ = nullptr;
    Vec3d point_;
};

} // namespace myfrontend
#endif