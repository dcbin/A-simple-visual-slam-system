#include "mappoint.h"

namespace myfrontend {

unsigned long MapPoint::factory_id_ = 0;

MapPoint::MapPoint() 
    : good_(true), id_(-1), pos_(Vec3d(0, 0, 0)),
    norm_(Vec3d(0, 0, 0)), observed_times_(0),
    matched_times_(0){}
MapPoint::MapPoint(unsigned long id, const Vec3d& position = Vec3d(0, 0, 0),
                   const Vec3d& norm = Vec3d(0, 0, 0),
                   const cv::Mat &descriptor = cv::Mat(),
                   Frame::Ptr frame = nullptr)
    : good_(true), id_(id), pos_(position),
      norm_(norm), descriptor_(descriptor),
      observed_times_(0), matched_times_(0) {
        observed_frames_.emplace_back(frame);
    }

MapPoint::Ptr MapPoint::createMapPoint() {
    static unsigned long factory_id = 0;
    return MapPoint::Ptr(new MapPoint(factory_id++));
}

MapPoint::Ptr MapPoint::createMapPoint(const Vec3d& position_world,
                                       const Vec3d& norm,
                                       const cv::Mat &descriptor,
                                       Frame::Ptr frame) {
    return MapPoint::Ptr(new MapPoint(factory_id_++, position_world, norm, descriptor, frame));
}

cv::Point3f MapPoint::getPositionCV() const{
    return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
}
} // namespace myfrontend