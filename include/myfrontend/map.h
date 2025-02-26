#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "mappoint.h"
#include "frame.h"

namespace myfrontend {

class Map {
public:
    using Ptr = std::shared_ptr<Map>;
    std::unordered_map<unsigned long, MapPoint::Ptr> map_points_; // 存储所有地图点
    std::unordered_map<unsigned long, Frame::Ptr> keyframes_; // 存储所有关键帧

    Map() {}

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
};

} // namespace myfrontend

#endif // MAP_H