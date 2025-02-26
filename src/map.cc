#include "map.h"

namespace myfrontend {

void Map::insertKeyFrame(Frame::Ptr frame) {
    LOG(INFO) << "Key frame size = " << keyframes_.size();
    if (keyframes_.find(frame->id_) == keyframes_.end()) {
        keyframes_.insert({frame->id_, frame});
    } else {
        keyframes_[frame->id_] = frame;
    }
}

void Map::insertMapPoint(MapPoint::Ptr map_point) {
    if (map_points_.find(map_point->id_) == map_points_.end()) {
        map_points_.insert({map_point->id_, map_point});
    } else {
        map_points_[map_point->id_] = map_point;
    }
}
} // namespace myfrontend