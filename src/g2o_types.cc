#include "g2o_types.h"
using namespace myfrontend;

void EdgeProjectXYZ2UV::computeError() {
    // 获取第一个顶点
    const g2o::VertexPointXYZ *v0 = static_cast<g2o::VertexPointXYZ*>(_vertices[0]);
    // 获取第二个顶点
    const g2o::VertexSE3Expmap *v1 = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
    Vec3d point_3d = v0->estimate();
    g2o::SE3Quat T(v1->estimate());
    Vec3d p_cam = T.map(point_3d);
    Vec2d p_uv = camera_->camera2pixel(p_cam);
    _error = _measurement - p_uv;
}

void EdgeProjectXYZ2UV::linearizeOplus() {
    // 获取第一个顶点
    const g2o::VertexPointXYZ *v0 = static_cast<g2o::VertexPointXYZ*>(_vertices[0]);
    // 获取第二个顶点
    const g2o::VertexSE3Expmap *v1 = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
    Vec3d point_3d = v0->estimate();
    g2o::SE3Quat T(v1->estimate());
    Vec3d p_cam = T.map(point_3d);


    double x = p_cam[0];
    double y = p_cam[1];
    double z_inv = 1.0 / p_cam[2];
    double z_inv_2 = z_inv * z_inv;
    
    Eigen::Matrix<double, 2, 3> mat;
    mat << -camera_->fx_ * z_inv, 0, camera_->fx_ * x * z_inv_2, 
           0, -camera_->fy_ * z_inv, camera_->fy_ * y * z_inv_2;
    Sophus::SE3d T_mat(T.to_homogeneous_matrix());
    Eigen::Matrix3d R = T_mat.rotationMatrix();
    // 误差对3D点的雅可比矩阵
    _jacobianOplusXi = mat * R;
    // 误差对相机位姿的雅可比矩阵
    _jacobianOplusXj(0, 0) = -camera_->fx_ * z_inv;
    _jacobianOplusXj(0, 1) = 0;
    _jacobianOplusXj(0, 2) = camera_->fx_ * x * z_inv_2;
    _jacobianOplusXj(0, 3) = camera_->fx_ * x * y * z_inv_2;
    _jacobianOplusXj(0, 4) = -camera_->fx_ * (1 + x * x * z_inv_2);
    _jacobianOplusXj(0, 5) = camera_->fx_ * y * z_inv;
    _jacobianOplusXj(1, 0) = 0;
    _jacobianOplusXj(1, 1) = -camera_->fy_ * z_inv;
    _jacobianOplusXj(1, 2) = camera_->fy_ * y * z_inv_2;
    _jacobianOplusXj(1, 3) = camera_->fy_ * (1 + y * y * z_inv_2);
    _jacobianOplusXj(1, 4) = -camera_->fy_ * x * y * z_inv_2;
    _jacobianOplusXj(1, 5) = -camera_->fy_ * x * z_inv;
}