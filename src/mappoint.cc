#include "mappoint.h"

namespace myfrontend {

MapPoint::MapPoint() 
    : id_(-1), pos_(Vec3d(0, 0, 0)),
    norm_(Vec3d(0, 0, 0)), observed_times_(0),
    correct_times_(0){}
MapPoint::MapPoint(unsigned long id, const Vec3d& position = Vec3d(0, 0, 0),
                   const Vec3d& norm = Vec3d(0, 0, 0))
    : id_(id), pos_(position), norm_(norm), observed_times_(0), correct_times_(0) {}

MapPoint::Ptr MapPoint::createMapPoint() {
    static unsigned long factory_id = 0;
    return MapPoint::Ptr(new MapPoint(factory_id++));
}
} // namespace myfrontend