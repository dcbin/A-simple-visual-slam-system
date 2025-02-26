#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "camera.h"
namespace myfrontend {

class Frame {
public:
    using Ptr = std::shared_ptr<Frame>;
    unsigned long id_; // 当前帧的id
    double time_stamp_; // 当前帧的时间戳
    Sophus::SE3d T_c_w_; // 从世界坐标系到相机坐标系的变换矩阵
    Camera::Ptr camera_; // 相机模型
    cv::Mat color_; // RGB图像
    cv::Mat depth_; // 深度图像

public:

    Frame(): id_(-1), time_stamp_(-1), camera_(nullptr) {}
    Frame(long id, double time_stamp = 0,
          Sophus::SE3d T_c_w = Sophus::SE3d(),
          Camera::Ptr camera = nullptr,
          cv::Mat color = cv::Mat(),
          cv::Mat depth = cv::Mat())
        : id_(id), time_stamp_(time_stamp),
          T_c_w_(T_c_w), camera_(camera),
          color_(color), depth_(depth) {}
    ~Frame() {}

public:
    // 静态成员函数,创建一个新的帧;
    // 静态成员函数的作用是不需要实例化对象就可以调用;
    // 此函数属于类而不属于某个对象
    static Frame::Ptr createFrame(); 
    
    /**
     * @brief 根据深度图获取某个关键点的深度.
     *        若该点深度为0,则在四邻域寻找一个不为0的点的深度替代;
     *        若四邻域深度值都为0,则返回-1.
     * @param kp 关键点
     * @return double 深度值
     */
    double findDepth(const cv::KeyPoint& kp);
    
    // 获取相机光心在世界坐标系中的坐标
    Vec3d getCamPosition() const;
    
    // 检查世界坐标系中的一个点是否在相机的视野中
    bool isInFrame(const Vec3d& pt_world);
};

} // namespace myfrontend
#endif // FRAME_H