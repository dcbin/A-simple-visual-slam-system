#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"
namespace myfrontend {
class Camera {

public:
    using Ptr = std::shared_ptr<Camera>;
    float fx_, fy_, cx_, cy_, depth_scale_;

public:
    // 无参构造函数,适用于需要在后续设置参数的情况
    Camera();
    // 有参构造函数,适用于已知参数的情况
    Camera(float fx, float fy, float cx, float cy, float depth_scale)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}
    ~Camera(){}

    /**
     * @brief 世界坐标系到相机坐标系的转换
     * @param p_w 世界坐标系下的点
     * @param T_c_w 世界坐标系到相机坐标系的变换矩阵
     * @return Vec3d 相机坐标系下的点
     */
    Vec3d world2camera(const Vec3d &p_w, const Sophus::SE3d &T_c_w);

    /**
     * @brief 相机坐标系到世界坐标系的转换
     * @param p_c 相机坐标系下的点
     * @param T_c_w 世界坐标系到相机坐标系的变换矩阵
     * @return Vec3d 世界坐标系下的点
     */
    Vec3d camera2world(const Vec3d &p_c, const Sophus::SE3d &T_c_w);

    /**
     * @brief 相机坐标系到像素坐标系的转换
     * @param p_c 相机坐标系下的点
     * @return Vec2d 像素坐标系下的点
     */
    Vec2d camera2pixel(const Vec3d &p_c);

    /**
     * @brief 像素坐标系到相机坐标系的转换
     * @param p_p 像素坐标系下的点
     * @param depth 深度值
     * @return Vec3d 相机坐标系下的点
     */
    Vec3d pixel2camera(const Vec2d &p_p, double depth = 1);

    /**
     * @brief 像素坐标系到世界坐标系的转换
     * @param p_p 像素坐标系下的点
     * @param T_c_w 世界坐标系到相机坐标系的变换矩阵
     * @param depth 深度值
     * @return Vec3d 世界坐标系下的点
     */
    Vec3d pixel2world(const Vec2d &p_p, const Sophus::SE3d &T_c_w, double depth = 1);
    
    /**
     * @brief 世界坐标系到像素坐标系的转换
     * @param p_w 世界坐标系下的点
     * @param T_c_w 世界坐标系到相机坐标系的变换矩阵
     * @return Vec2d 像素坐标系下的点
     */
    Vec2d world2pixel(const Vec3d &p_w, const Sophus::SE3d &T_c_w);
};
} // namespace myfrontend

#endif // CAMERA_H