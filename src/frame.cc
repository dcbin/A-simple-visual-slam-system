#include "frame.h"

namespace myfrontend {

Frame::Ptr Frame::createFrame() {
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++));
}

double Frame::findDepth(const cv::KeyPoint& kp) {
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x]; // 深度图中的深度值
    if (d != 0) 
    {
        return double(d) / camera_->depth_scale_;
    } 
    else 
    {
        // 用邻域内的某个点的深度值代替
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; i++) {
            d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (d != 0) {
                return double(d) / camera_->depth_scale_;
            }
        }
    }
    // LOG(ERROR) << "该点及其四邻域内无深度信息!";
    return -1.0;
}

Vec3d Frame::getCamPosition() const {
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame(const Vec3d& pt_world) {
    Vec3d p_cam = camera_->world2camera(pt_world, T_c_w_);
    if (p_cam(2, 0) < 0) {
        return false;
    }
    Vec2d pixel = camera_->camera2pixel(p_cam);
    return pixel(0, 0) > 0 && pixel(1, 0) > 0 && pixel(0, 0) < color_.cols && pixel(1, 0) < color_.rows;
}

} // namespace myfrontend