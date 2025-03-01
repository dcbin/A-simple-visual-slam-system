#include "odometry.h"
#include <chrono>
#include "g2o_types.h"

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
    std::vector<cv::Point3f> pts3d; // 参考帧的相机坐标系下的3D点
    std::vector<cv::Point2f> pts2d; // 当前帧的像素坐标系下的2D点

    std::vector<Vec3d> pts_world_eigen;
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
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 10, 8.0, 0.99, inliers);
    num_inliers_ = inliers.rows;  // 更新内点数量
    LOG(ERROR) << "pnp inliers: " << num_inliers_;
    Eigen::Vector3d rvec_eigen(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
    T_c_r_estimated_ = Sophus::SE3d(
        Sophus::SO3d::exp(rvec_eigen),
        Vec3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
    );
    LOG(ERROR) << "pnp解算的变换矩阵: \n" << T_c_r_estimated_.matrix();
    poseAndPoint(pts3d, pts2d, inliers);
    // onlyPose(pts3d, pts2d, inliers);
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
    pose->setEstimate(g2o::SE3Quat(T_c_r_estimated_.rotationMatrix(),
                                   T_c_r_estimated_.translation()));
    
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
    T_c_r_estimated_ = Sophus::SE3d(
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
    optimizer_lh.setVerbose(true); // 开启优化过程输出
    // 向优化器中添加顶点
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    // 由于pnp的的初值已经很好,所以优化过程中的误差下降不会太明显
    pose->setEstimate(g2o::SE3Quat(T_c_r_estimated_.rotationMatrix(),
                                   T_c_r_estimated_.translation()));
    
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
    LOG(ERROR) << "共" << optimizer_lh.edges().size() << "条边.";
    // 初始化
    optimizer_lh.initializeOptimization();
    // 最大迭代次数
    optimizer_lh.optimize(10);
    T_c_r_estimated_ = Sophus::SE3d(
                       pose->estimate().rotation(),
                       pose->estimate().translation());
    LOG(ERROR) << "BA后的位姿:\n" << T_c_r_estimated_.matrix();
}
} // namespace myfrontend