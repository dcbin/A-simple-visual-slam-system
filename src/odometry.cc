#include "odometry.h"
#include <chrono>

namespace myfrontend {

Odometry::Odometry():state_(INITIALIZING), ref_(nullptr),
                     curr_(nullptr), map_(new Map), num_inliers_(0),
                     num_lost_(0) {
    num_features_ = Config::getParam<int>("num_features");
    num_inliers_min_ = Config::getParam<int>("num_inliers_min");
    key_frame_min_rot_ = Config::getParam<double>("key_frame_min_rot");
    key_frame_min_trans_ = Config::getParam<double>("key_frame_min_trans");
    level_pyramid_ = Config::getParam<int>("level_pyramid");
    scale_factor_ = Config::getParam<float>("scale_factor");
    match_ratio_ = Config::getParam<int>("match_ratio");
    depth_scale_ = Config::getParam<float>("depth_scale");
    num_lost_max_ = Config::getParam<int>("num_lost_max");
    orb_ = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
}

void Odometry::extractKeyPoints() {
    orb_->detect(curr_->color_, keypoints_curr_);
}

void Odometry::computeDescriptors() {
    orb_->compute(curr_->color_, keypoints_curr_, descriptor_curr_);
}

void Odometry::featureMatch() {
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches; // 存放初始的匹配结果
    matcher.match(descriptor_ref_, descriptor_curr_, matches);
    // 选取最优匹配
    float min_dis = std::min_element(matches.begin(), matches.end(),
        [](const cv::DMatch &m1, const cv::DMatch &m2) {
            return m1.distance < m2.distance;
        })->distance;

    feature_matches_.clear(); // 清空上一次的匹配结果
    for (cv::DMatch &m: matches) {
        if (m.distance < std::max<float>(min_dis * 2.0, 30.0)) {
            feature_matches_.push_back(m);
        }
    }
    LOG(ERROR) << "good matches: " << feature_matches_.size();
}

void Odometry::poseEstimationPnP() {
    // 准备PnP求解所需的数据
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;
    for (cv::DMatch &m: feature_matches_) {
        pts3d.push_back(points_3d_ref_[m.queryIdx]);
        pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
    }
    if(pts2d.size() == pts3d.size()) {
        LOG(ERROR) << "共有" << pts3d.size() << "对匹配点";
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0, 0, 1
    );
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;  // 更新内点数量
    LOG(ERROR) << "pnp inliers: " << num_inliers_;
    Eigen::Vector3d rvec_eigen(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
    T_c_r_estimated_ = Sophus::SE3d(
        Sophus::SO3d::exp(rvec_eigen),
        Vec3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
    );
}

bool Odometry::checkEstimatedPose() {
    if(num_inliers_ < num_inliers_min_) {
        LOG(ERROR) << "内点数量过少,丢弃此次估计!";
        return false;
    }

    if(T_c_r_estimated_.log().norm() > 5.0) {
        LOG(ERROR) << "相机位移异常偏大,丢弃此次估计!";
        return false;
    }
    return true;
}

bool Odometry::checkKeyFrame() {
    Sophus::SO3d R = T_c_r_estimated_.rotationMatrix();
    Sophus::Vector3d rot_vec = R.log();
    Sophus::Vector3d trans_vec = T_c_r_estimated_.translation();

    if(rot_vec.norm() < key_frame_min_rot_ &&
       trans_vec.norm() < key_frame_min_trans_) {
        LOG(ERROR) << "相机运动过小,并非关键帧!";
        return false;
    }
    return true;
}

void Odometry::setRef3DPoints() {
    points_3d_ref_.clear();
    LOG(ERROR) << "进入setRef3DPoints()";
    // 清空参考帧的特征描述子
    descriptor_ref_ = cv::Mat().clone();
    for (int i = 0; i < keypoints_curr_.size(); i++) {
        float x = keypoints_curr_[i].pt.x;
        float y = keypoints_curr_[i].pt.y;

        Vec2d pts_uv(x, y);
        double depth = curr_->findDepth(keypoints_curr_[i]);
        if (depth <= 0 || !std::isfinite(depth)) {
            continue; // 跳过深度丢失或错误的特征点
        }
        Vec3d point_3d_ref = curr_->camera_->pixel2camera(pts_uv, depth);
        cv::Point3f point(point_3d_ref(0, 0),
                          point_3d_ref(1, 0),
                          point_3d_ref(2, 0));
        points_3d_ref_.emplace_back(point);
        descriptor_ref_.push_back(descriptor_curr_.row(i));
    }
    LOG(ERROR) << "参考帧的3D点数量:" << points_3d_ref_.size();
}

bool Odometry::addFrame(Frame::Ptr &frame) {
    if(state_ == Odometry::odometry_state::INITIALIZING){
        ref_ = curr_ = frame;
        ref_->T_c_w_ = Sophus::SE3d(); // 这里ref_和curr_指针指向一致
        T_c_r_estimated_ = Sophus::SE3d();
        timer t1 = std::chrono::steady_clock::now();
        extractKeyPoints();
        timer t2 = std::chrono::steady_clock::now();
        computeDescriptors();
        LOG(ERROR) << "当前描述子尺寸:" << descriptor_curr_.size();
        timer t3 = std::chrono::steady_clock::now();
        setRef3DPoints();
        addKeyFrame();
        state_ = odometry_state::OK;
        LOG(ERROR) << "里程计初始化成功!\n"
                   << "提取关键点用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()
                   << "\n 计算描述子用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();    
    }
    else if(state_ == odometry_state::OK) {
        curr_ = frame;
        timer t1 = std::chrono::steady_clock::now();
        extractKeyPoints();
        timer t2 = std::chrono::steady_clock::now();
        computeDescriptors();
        timer t3 = std::chrono::steady_clock::now();
        featureMatch();
        timer t4 = std::chrono::steady_clock::now();
        poseEstimationPnP();
        timer t5 = std::chrono::steady_clock::now();
        LOG(ERROR) << "提取关键点用时:"
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()
                   << "\n 计算描述子用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()
                   << "\n 特征匹配用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()
                   << "\n pnp解算用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t4).count();         
        if(checkEstimatedPose()) {
            curr_->T_c_w_  = T_c_r_estimated_ * ref_->T_c_w_;
            num_lost_ = 0;
            setRef3DPoints();
            ref_ = curr_;
            if(checkKeyFrame()) {
                addKeyFrame();
            }
        }
        else {
            num_lost_++;
            LOG(ERROR) << "Bad estimation!";
            if (num_lost_ >= num_lost_max_) {
                state_ = odometry_state::LOST;
            }
            return false;
        }
    }
    else if(state_ == odometry_state::LOST) {
        LOG(ERROR) << "里程计已经丢失!以当前位置为起始点重置里程计!";
        state_ = odometry_state::INITIALIZING;
    }
    return true;
}

void Odometry::addKeyFrame() {
    map_->insertKeyFrame(curr_);
}

} // namespace myfrontend