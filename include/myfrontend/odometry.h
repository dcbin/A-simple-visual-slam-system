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
    std::vector<MapPoint::Ptr> matched_3dpts_; // 匹配到的3D点
    std::vector<int> matched_2dkp_index_; // 匹配到的2D点
    Frame::Ptr last_frame_; // 上一帧
    Frame::Ptr curr_; // 当前帧
    cv::Mat descriptor_map_; // 局部地图的特征描述子
    cv::Mat descriptor_curr_; // 当前帧ORB特征描述子，需要计算
    std::vector<cv::KeyPoint> keypoints_curr_; // 当前帧中的特征点，需要计算
    cv::Ptr<cv::ORB> orb_; // orb特征提取器
    std::vector<cv::DMatch> feature_matches_; // 特征匹配结果
    Sophus::SE3d T_c_w_estimated_; // 估计结果
    Map::Ptr map_; // 地图

    int num_features_; // 特征点数量

    int num_inliers_; // 内点数量
    int num_inliers_min_; // 最小内点数量
    int num_lost_; // 丢帧数量
    int num_lost_max_; // 最大允许丢帧数量
    int iter_max_; // BA优化的最大迭代次数

    double key_frame_min_rot_; // 旋转阈值
    double key_frame_min_trans_; // 平移阈值

    int level_pyramid_; // 规定ORB特征提取器中的金字塔层数
    float scale_factor_; // 规定ORB特征提取器中的尺度因子
    int match_ratio_; // 特征匹配比例

    /**
     * @brief 地图点清除比例(被匹配次数 / 被观测次数);
     * 如果这个值太小,会导致地图点过多,计算量大,而且很多差的地图点容易赖着不走;
     * 如果这个值太大,会导致局部地图更新很快,丢失很多地图点
     */
    double  map_point_erase_ratio_; // 地图点清除比例
    int map_point_erase_min_; // 最小地图点数量
    int map_point_erase_max_; // 最大地图点数量
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
    // 匹配局部地图和当前帧的ORB特征
    void featureMatch();
    // 用PnP解算参考帧与当前帧的相对位姿
    void poseEstimationPnP();

    // 检查求解的位姿是否合理
    bool checkEstimatedPose();
    
    /**
     * @brief 检查当前帧是否可以作为关键帧.若旋转和平移的程度大于某个阈值,
     * 则可以作为关键帧;若旋转和平移的尺度非常小,则认为相机没有运动,不作为关键帧
     */
    bool checkKeyFrame();

    // 地图优化
    void optimizeMap();

    // 把当前帧加入地图(如果是关键帧的话)
    void addKeyFrame();

    // 获取同一个地图点在两帧间的视角
    double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);

    // 把当前帧中未匹配到的点加入地图点中
    void addMapPoints();
    // 同时优化相机位姿和3D点坐标
    void poseAndPoint(const std::vector<cv::Point3f> &pts3d,
                                const std::vector<cv::Point2f> &pts2d, 
                                const cv::Mat &inliers);
    // 只优化相机位姿
    void onlyPose(const std::vector<cv::Point3f> &pts3d,
                  const std::vector<cv::Point2f> &pts2d, 
                  const cv::Mat &inliers);
};

} // namespace myfrontend

#endif // ODOMETRY_H