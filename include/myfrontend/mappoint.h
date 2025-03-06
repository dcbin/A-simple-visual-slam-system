#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"
#include "frame.h"
namespace myfrontend {
class MapPoint {
public:
    using Ptr = std::shared_ptr<MapPoint>;
    unsigned long id_; // ID
    bool good_; // 是否是一个好的点
    static unsigned long factory_id_;
    Vec3d pos_; // 在世界坐标系中的位置
    Vec3d norm_; // 视觉中的法线
    cv::Mat descriptor_; // 路标点的描述子
    std::list<Frame::Ptr> observed_frames_; // 观测到该路标点的帧
    int observed_times_; // 被观测次数
    int matched_times_; // 被匹配次数

public:
    // 无参构造函数
    MapPoint();
    // 有参构造函数
    MapPoint(unsigned long id, const Vec3d& position, const Vec3d& norm, const cv::Mat &descriptor, Frame::Ptr frame);
    ~MapPoint() {}
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(const Vec3d& position_world,
                                        const Vec3d& norm,
                                        const cv::Mat &descriptor,
                                        Frame::Ptr frame);
    // 返回当前路标点的cv::Point3f格式的世界坐标
    cv::Point3f getPositionCV() const;
};

} // namespace myfrontend

#endif // MAPPOINT_H