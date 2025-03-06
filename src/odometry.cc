#include "odometry.h"
#include <chrono>
#include "g2o_types.h"

namespace myfrontend {

Odometry::Odometry():state_(INITIALIZING),last_frame_(nullptr), curr_(nullptr),
                     map_(new Map), num_inliers_(0), num_lost_(0) {
    num_features_ = Config::getParam<int>("num_features");
    num_inliers_min_ = Config::getParam<int>("num_inliers_min");
    key_frame_min_rot_ = Config::getParam<double>("key_frame_min_rot");
    key_frame_min_trans_ = Config::getParam<double>("key_frame_min_trans");
    level_pyramid_ = Config::getParam<int>("level_pyramid");
    scale_factor_ = Config::getParam<float>("scale_factor");
    match_ratio_ = Config::getParam<int>("match_ratio");
    depth_scale_ = Config::getParam<float>("depth_scale");
    num_lost_max_ = Config::getParam<int>("num_lost_max");
    iter_max_ = Config::getParam<int>("iter_max");
    map_point_erase_ratio_ = Config::getParam<double>("map_point_erase_ratio");
    map_point_erase_min_ = Config::getParam<int>("map_point_erase_min");
    map_point_erase_max_ = Config::getParam<int>("map_point_erase_max");
    orb_ = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
}

void Odometry::extractKeyPoints() {
    orb_->detect(curr_->color_, keypoints_curr_);
}

void Odometry::computeDescriptors() {
    orb_->compute(curr_->color_, keypoints_curr_, descriptor_curr_);
}

void Odometry::featureMatch() {
    descriptor_map_ = cv::Mat().clone();
    // 存放局部地图中在当前帧视野范围内的点
    std::vector<MapPoint::Ptr> candidate_mappoints_;
    std::vector<unsigned long> map_point_index;
    /**
     * @brief 从局部地图中筛选在当前帧视野范围内的点，并记录它的描述子
     */
    for(auto mappoint : map_->map_points_) {
        const auto &p = mappoint.second;
        if(curr_->isInFrame(p->pos_)) {
            p->observed_times_++;
            p->observed_frames_.emplace_back(curr_);
            candidate_mappoints_.emplace_back(p);
            map_point_index.emplace_back(p->id_);
            descriptor_map_.push_back(p->descriptor_);
        }
    }
    LOG(ERROR) << "视野范围内的地图点数量: " << candidate_mappoints_.size();
    LOG(ERROR) << "局部地图描述子数量: " << descriptor_map_.rows;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches; // 存放初始的匹配结果
    matcher.match(descriptor_map_, descriptor_curr_, matches);

    // 选取最优匹配
    float min_dis = std::min_element(matches.begin(), matches.end(),
        [](const cv::DMatch &m1, const cv::DMatch &m2) {
            return m1.distance < m2.distance;
        })->distance;

    feature_matches_.clear(); // 清空上一次的匹配结果
    matched_3dpts_.clear();
    matched_2dkp_index_.clear();
    for (cv::DMatch &m: matches) {
        if (m.distance < std::max<float>(min_dis * 2.0, 30.0)) {
            feature_matches_.push_back(m);
            matched_3dpts_.push_back(candidate_mappoints_[m.queryIdx]);
            map_->map_points_[map_point_index[m.queryIdx]]->matched_times_++;
            matched_2dkp_index_.push_back(m.trainIdx);
        }
    }
}

void Odometry::poseEstimationPnP() {
    // 准备PnP求解所需的数据
    std::vector<cv::Point3f> pts3d; // 局部地图世界坐标系下的3D点
    std::vector<cv::Point2f> pts2d; // 当前帧的像素坐标系下的2D点

    for(auto &m : matched_3dpts_) {
        pts3d.push_back(m->getPositionCV());
    }
    for(auto &m : matched_2dkp_index_) {
        pts2d.push_back(keypoints_curr_[m].pt);
    }

    if(pts2d.size() == pts3d.size()) {
        LOG(ERROR) << "共有" << pts3d.size() << "对匹配点";
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        curr_->camera_->fx_, 0, curr_->camera_->cx_,
        0, curr_->camera_->fy_, curr_->camera_->cy_,
        0, 0, 1);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 10, 8.0, 0.99, inliers);
    num_inliers_ = inliers.rows;  // 更新内点数量
    // LOG(ERROR) << "pnp inliers: " << num_inliers_;
    Eigen::Vector3d rvec_eigen(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
    T_c_w_estimated_ = Sophus::SE3d(
        Sophus::SO3d::exp(rvec_eigen),
        Vec3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
    );
    LOG(ERROR) << "pnp解算的变换矩阵: \n" << T_c_w_estimated_.matrix();
    // poseAndPoint(pts3d, pts2d, inliers);
    onlyPose(pts3d, pts2d, inliers);
}

bool Odometry::checkEstimatedPose() {
    if(num_inliers_ < num_inliers_min_) {
        LOG(ERROR) << "内点数量过少,丢弃此次估计!";
        return false;
    }
    Sophus::SE3d T_c_r_ = last_frame_->T_c_w_ * T_c_w_estimated_.inverse();
    if(T_c_r_.log().norm() > 2.0) {
        LOG(ERROR) << "相机运动异常偏大,丢弃此次估计!";
        return false;
    }
    return true;
}

bool Odometry::checkKeyFrame() {
    Sophus::SE3d T_c_r = last_frame_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::SO3d R(T_c_r.rotationMatrix());
    Sophus::Vector3d rot_vec = R.log();
    Sophus::Vector3d trans_vec = T_c_r.translation();

    if(rot_vec.norm() < key_frame_min_rot_ &&
       trans_vec.norm() < key_frame_min_trans_) {
        LOG(ERROR) << "相机运动过小,并非关键帧!";
        return false;
    }
    return true;
}

bool Odometry::addFrame(Frame::Ptr &frame) {
    if(state_ == Odometry::odometry_state::INITIALIZING){
        curr_ = last_frame_ = frame;
        T_c_w_estimated_ = Sophus::SE3d();
        last_frame_->T_c_w_ = T_c_w_estimated_;
        timer t1 = std::chrono::steady_clock::now();
        extractKeyPoints();
        timer t2 = std::chrono::steady_clock::now();
        computeDescriptors();
        timer t3 = std::chrono::steady_clock::now();
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
        curr_->T_c_w_ = last_frame_->T_c_w_;
        timer t1 = std::chrono::steady_clock::now();
        extractKeyPoints();
        timer t2 = std::chrono::steady_clock::now();
        computeDescriptors();
        timer t3 = std::chrono::steady_clock::now();
        featureMatch();
        LOG(ERROR) << "特征匹配完成!";
        timer t4 = std::chrono::steady_clock::now();
        poseEstimationPnP();
        timer t5 = std::chrono::steady_clock::now();
        LOG(ERROR) << "提取关键点用时:"
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()
                   << "\n 计算描述子用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()
                   << "\n 特征匹配用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()
                   << "\n 位姿估计用时:" 
                   << std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t4).count();         
        if(checkEstimatedPose()) {
            curr_->T_c_w_  = T_c_w_estimated_;
            optimizeMap();
            num_lost_ = 0;
            last_frame_ = curr_;
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
    if(map_->keyframes_.empty()) {
        for(int i = 0; i < keypoints_curr_.size(); i++) {
            cv::KeyPoint key_point = keypoints_curr_[i];
            double depth = curr_->findDepth(key_point);
            if(depth <= 0)
                continue;
            Vec2d p_uv(key_point.pt.x, key_point.pt.y);
            Vec3d p_w = last_frame_->camera_->pixel2world(p_uv, T_c_w_estimated_, depth);
            Vec3d n = p_w - last_frame_->getCamPosition();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(p_w, n, descriptor_curr_.row(i).clone(), last_frame_);
            map_point->observed_times_++;
            map_->insertMapPoint(map_point);
        }
        LOG(ERROR) << "第一帧中的所有特征点作为路标点!";
    }
    map_->insertKeyFrame(curr_);
    last_frame_ = curr_;
}

double Odometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point) {
    Vec3d n = point->pos_ - frame->getCamPosition();
    n.normalize();
    return acos(n.transpose() * point->norm_);
}

void Odometry::addMapPoints() {
    std::vector<bool> matched(keypoints_curr_.size(), false);
    for(auto &m : feature_matches_) {
        matched[m.trainIdx] = true;
    }
    for(int i = 0; i < keypoints_curr_.size(); i++) {
        if(matched[i])
            continue;
        double depth = curr_->findDepth(keypoints_curr_[i]);
        if(depth <= 0)
            continue;
        Vec2d p_uv(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y);
        Vec3d p_w = curr_->camera_->pixel2world(p_uv, T_c_w_estimated_, depth);
        Vec3d n = p_w - curr_->getCamPosition();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(p_w, n, descriptor_curr_.row(i).clone(), curr_);
        map_point->observed_times_++;
        map_->insertMapPoint(map_point);
    }
}

void Odometry::optimizeMap() {
    for(auto map_point_iter = map_->map_points_.begin(); map_point_iter != map_->map_points_.end();) {
        if(!curr_->isInFrame(map_point_iter->second->pos_)) {
            map_point_iter = map_->map_points_.erase(map_point_iter);
            continue;
        }
        
        float match_ratio = float(map_point_iter->second->matched_times_) /
                            map_point_iter->second->observed_times_;
        if(match_ratio < map_point_erase_ratio_) {
            map_point_iter = map_->map_points_.erase(map_point_iter);
            continue;
        }
        double angle = getViewAngle(curr_, map_point_iter->second);
        if(angle > M_PI / 6.0) {
            map_point_iter = map_->map_points_.erase(map_point_iter);
            continue;
        }
        map_point_iter++;
    }

    if(map_->map_points_.size() < map_point_erase_min_) {
        addMapPoints();
    }
    if(map_->map_points_.size() > map_point_erase_max_) {
        map_point_erase_ratio_ += 0.05;
    }
    else {
        map_point_erase_ratio_ = Config::getParam<double>("map_point_erase_ratio");
    }
}

void Odometry::poseAndPoint(const std::vector<cv::Point3f> &pts3d,
                        const std::vector<cv::Point2f> &pts2d, 
                        const cv::Mat &inliers) {
    // 使用BA优化相机位姿和特征点位置,配置g2o
    using BlockSolver = g2o::BlockSolver<g2o::BlockSolver_6_3>;
    using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
    using optimizer = g2o::OptimizationAlgorithmLevenberg;
    auto linear_solver = std::make_unique<LinearSolver>();
    auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));
    auto solver = new optimizer(std::move(block_solver));
    g2o::SparseOptimizer optimizer_lh;
    optimizer_lh.setAlgorithm(solver);
    optimizer_lh.setVerbose(true); // 开启优化过程输出
    // 向优化器中添加顶点
    // ----1.添加相机位姿顶点----
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(T_c_w_estimated_.rotationMatrix(),
                                   T_c_w_estimated_.translation()));
    
    // 随机给一个偏差较大的初始值,更容易看到误差下降
    // Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    // Eigen::Vector3d t = Eigen::Vector3d::Identity() * 0.1;
    // pose->setEstimate(g2o::SE3Quat(R, t));
    optimizer_lh.addVertex(pose);
    // ----2.添加特征点顶点----
    for (size_t i = 0; i < inliers.rows; i++) {
        g2o::VertexPointXYZ *point = new g2o::VertexPointXYZ();
        int index = inliers.at<int>(i,0);
        point->setId(i + 1);
        point->setEstimate(Vec3d(pts3d[index].x,
                                 pts3d[index].y,
                                 pts3d[index].z));
        // point->setEstimate(Vec3d(pts3d[index].x,
        //                          pts3d[index].y,
        //                          1.0));
        point->setMarginalized(true);
        optimizer_lh.addVertex(point);
    }
    // 向优化器中添加边, 重投影误差
    for (size_t i = 0; i < inliers.rows; i++) {
        EdgeProjectXYZ2UV *edge = new EdgeProjectXYZ2UV();
        int index = inliers.at<int>(i, 0);
        // 注意这里的顶点顺序要与ComputeError()中的顺序一致
        edge->setId(i);
        edge->setVertex(0, optimizer_lh.vertex(i + 1));
        edge->setVertex(1, optimizer_lh.vertex(0));
        edge->setMeasurement(Vec2d(pts2d[index].x, pts2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel( new g2o::RobustKernelHuber());
        edge->camera_ = curr_->camera_.get();
        optimizer_lh.addEdge(edge);
    }
    // 初始化
    optimizer_lh.initializeOptimization();
    // 最大迭代次数
    optimizer_lh.optimize(10);
    T_c_w_estimated_ = Sophus::SE3d(
                       pose->estimate().rotation(),
                       pose->estimate().translation());
}

void Odometry::onlyPose(const std::vector<cv::Point3f> &pts3d,
                        const std::vector<cv::Point2f> &pts2d, 
                        const cv::Mat &inliers) {
    // 使用BA优化相机位姿
    using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>>;
    using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
    using optimizer = g2o::OptimizationAlgorithmLevenberg;
    auto linear_solver = std::make_unique<LinearSolver>();
    auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));
    auto solver = new optimizer(std::move(block_solver));
    g2o::SparseOptimizer optimizer_lh;
    optimizer_lh.setAlgorithm(solver);
    // optimizer_lh.setVerbose(true); // 开启优化过程输出
    // 向优化器中添加顶点
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    // 由于pnp的的初值已经很好,所以优化过程中的误差下降不会太明显
    pose->setEstimate(g2o::SE3Quat(T_c_w_estimated_.rotationMatrix(),
                                   T_c_w_estimated_.translation()));
    
    // 随机给一个偏差较大的初始值,更容易看到误差下降
    // Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    // Eigen::Vector3d t = Eigen::Vector3d::Identity();
    // pose->setEstimate(g2o::SE3Quat(R, t));
    optimizer_lh.addVertex(pose);
    // 向优化器中添加边, 重投影误差
    for (size_t i = 0; i < inliers.rows; i++) {
        EdgeProjectPoseOnly *edge = new EdgeProjectPoseOnly();
        int index = inliers.at<int>(i, 0);
        edge->setId(i);
        edge->setVertex(0, optimizer_lh.vertex(0));
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vec3d(pts3d[index].x,
                             pts3d[index].y,
                             pts3d[index].z);
        edge->setMeasurement(Vec2d(pts2d[index].x, pts2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel( new g2o::RobustKernelHuber());
        optimizer_lh.addEdge(edge);
    }
    // 初始化
    optimizer_lh.initializeOptimization();
    // 最大迭代次数
    optimizer_lh.optimize(iter_max_);
    T_c_w_estimated_ = Sophus::SE3d(
                       pose->estimate().rotation(),
                       pose->estimate().translation());
    LOG(ERROR) << "BA后的位姿:\n" << T_c_w_estimated_.matrix();
}
} // namespace myfrontend