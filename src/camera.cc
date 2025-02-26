#include "camera.h"
#include "config.h"

namespace myfrontend {
Camera::Camera() {
    fx_ = Config::getParam<double>("camera.fx");
    fy_ = Config::getParam<double>("camera.fy");
    cx_ = Config::getParam<double>("camera.cx");
    cy_ = Config::getParam<double>("camera.cy");
    depth_scale_ = Config::getParam<double>("camera.depth_scale");
}

Vec2d Camera::camera2pixel(const Vec3d &p_c) {
    return Vec2d(fx_ * p_c(0, 0) / p_c(2, 0) + cx_, fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vec3d Camera::pixel2camera(const Vec2d &p_p, double depth) {
    return Vec3d((p_p(0, 0) - cx_) * depth / fx_, (p_p(1, 0) - cy_) * depth / fy_, depth);
}

Vec3d Camera::pixel2world(const Vec2d &p_p, const Sophus::SE3d &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

Vec3d Camera::world2camera(const Vec3d &p_w, const Sophus::SE3d &T_c_w) {
    return T_c_w * p_w;
}

Vec3d Camera::camera2world(const Vec3d &p_c, const Sophus::SE3d &T_c_w) {
    return T_c_w.inverse() * p_c;
}

Vec2d Camera::world2pixel(const Vec3d &p_w, const Sophus::SE3d &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}
} // namespace myfrontend