#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "common_include.h"
#include "map.h"
#include "config.h"

namespace myfrontend {

class Odometry {
public:
    using Ptr = std::shared_ptr<Odometry>;
    using timer = std::chrono::steady_clock::time_point;
    enum odometry_state {
        INITIALIZING = -1,
        OK = 0,
        LOST = 1
    };
    odometry_state state_;
    Frame::Ptr ref_; // 参考帧
    Frame::Ptr curr_; // 当前帧
    cv::Mat descriptor_ref_; // 参考帧ORB特征描述子，是已知的
    cv::Mat descriptor_curr_; // 当前帧ORB特征描述子，需要计算
    std::vector<cv::Point3f> points_3d_ref_; // 参考帧中的特征点3D坐标，是已知的
    std::vector<cv::KeyPoint> keypoints_curr_; // 当前帧中的特征点，需要计算
    cv::Ptr<cv::ORB> orb_; // orb特征提取器
    std::vector<cv::DMatch> feature_matches_; // 特征匹配结果
    Sophus::SE3d T_c_r_estimated_; // 估计的相机位姿
    Map::Ptr map_; // 地图

    int num_features_; // 特征点数量

    int num_inliers_; // 内点数量
    int num_inliers_min_; // 最小内点数量
    int num_lost_; // 丢帧数量
    int num_lost_max_; // 最大允许丢帧数量

    double key_frame_min_rot_; // 旋转阈值
    double key_frame_min_trans_; // 平移阈值

    int level_pyramid_; // 规定ORB特征提取器中的金字塔层数
    float scale_factor_; // 规定ORB特征提取器中的尺度因子
    int match_ratio_; // 特征匹配比例

    float depth_scale_; // 相机的深度尺度

public:
    Odometry();
    ~Odometry(){}

    /**
     * @brief 根据当前帧和参考帧解算相机位姿
     * @param frame 当前帧
     * @return bool
     */
    bool addFrame(Frame::Ptr &frame);

protected:
    // 从当前帧的RGB图中提取ORB关键点
    void extractKeyPoints();
    // 计算当前帧中特征点的ORB特征描述子
    void computeDescriptors();
    // 匹配参考帧和当前帧的ORB特征
    void featureMatch();
    // 用PnP解算参考帧与当前帧的相对位姿
    void poseEstimationPnP();
    // 将当前帧的特征点转化为3D坐标,并将其设置为参考帧的3D坐标
    void setRef3DPoints();

    // 检查求解的位姿是否合理
    bool checkEstimatedPose();
    
    /**
     * @brief 检查当前帧是否可以作为关键帧.若旋转和平移的程度大于某个阈值,
     * 则可以作为关键帧;若旋转和平移的尺度非常小,则认为相机没有运动,不作为关键帧
     */
    bool checkKeyFrame();

    // 把当前帧加入地图(如果是关键帧的话)
    void addKeyFrame();
};

} // namespace myfrontend

#endif // ODOMETRY_H