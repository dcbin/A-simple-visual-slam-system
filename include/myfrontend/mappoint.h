#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"

namespace myfrontend {
class MapPoint {
public:
    using Ptr = std::shared_ptr<MapPoint>;
    unsigned long id_; // ID
    Vec3d pos_; // 在世界坐标系中的位置
    Vec3d norm_; // 视觉中的法线
    cv::Mat descriptor_; // 描述子
    int observed_times_; // 被观测次数
    int correct_times_; // 被匹配次数

public:
    // 无参构造函数
    MapPoint();
    // 有参构造函数
    MapPoint(unsigned long id, const Vec3d& position, const Vec3d& norm);
    ~MapPoint() {}

    static MapPoint::Ptr createMapPoint();
};

} // namespace myfrontend

#endif // MAPPOINT_H