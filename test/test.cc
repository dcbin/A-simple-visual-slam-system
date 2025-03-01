#include "mappoint.h"
#include "config.h"
#include "camera.h"
#include "frame.h"
#include "map.h"
#include "odometry.h"
#include "g2o_types.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/viz.hpp>


using namespace myfrontend;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    if (argc != 2) {
        LOG(ERROR) << "Usage: test parameter_file";
        return 1;
    }
    std::string config_file_path = argv[1];
    LOG(ERROR) << "Config file path: " << config_file_path;
    Config::setParameterFile(argv[1]); // 设置配置文件路径

    // 相机内参设置
    Camera::Ptr camera(new Camera);
    camera->fx_ = Config::getParam<double>("camera.fx");
    camera->fy_ = Config::getParam<double>("camera.fy");
    camera->cx_ = Config::getParam<double>("camera.cx");
    camera->cy_ = Config::getParam<double>("camera.cy");
    camera->depth_scale_ = Config::getParam<double>("camera.depth_scale");
    LOG(ERROR) << "相机内参读取成功!";

    std::string root_dir, rgb_dir, depth_dir;
    root_dir = Config::getParam<std::string>("dataset_dir");
    rgb_dir = root_dir + "/rgb";
    depth_dir = root_dir + "/depth";
    std::fstream associate_file(root_dir + "/" + "associate.txt");
    if (!associate_file.is_open()) {
        LOG(ERROR) << "无法打开associate.txt文件!";
        return 1;
    }
    std::string line;
    std::vector<double> rgb_timestamps, depth_timestamps;
    std::vector<std::string> rgb_paths, depth_paths;
    while (std::getline(associate_file, line)) {
        // 使用 istringstream 处理这一行数据
        std::istringstream ss(line);
        double rgb_timestamp, depth_timestamp;
        std::string rgb_path, depth_path;
        // 逐个读取每一项数据
        if(!(ss >> rgb_timestamp >> rgb_path >> depth_timestamp >> depth_path)) {
            LOG(ERROR) << "数据不完整,跳过该行!";
            continue;
        }
        rgb_timestamps.emplace_back(rgb_timestamp);
        depth_timestamps.emplace_back(depth_timestamp);
        rgb_paths.emplace_back(root_dir + '/' + rgb_path);
        depth_paths.emplace_back(root_dir + '/' + depth_path);
    }
    // 关闭文件
    associate_file.close();
    assert(rgb_timestamps.size() == rgb_paths.size() &&
           rgb_paths.size() == depth_timestamps.size() &&
           depth_timestamps.size() == depth_paths.size());
    LOG(ERROR) << "共有" << rgb_paths.size() << "帧图像";


    Odometry::Ptr odometry(new Odometry);
    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );
    double time_cost_total = 0; // 记录里程计处理所有帧的用时
    // 计算相机运动
    for(int i = 0; i < rgb_timestamps.size(); i++) {
        Frame::Ptr curr_frame = Frame::createFrame();
        curr_frame->camera_ = camera;
        curr_frame->color_ = cv::imread(rgb_paths[i], cv::IMREAD_COLOR);
        curr_frame->depth_ = cv::imread(depth_paths[i], cv::IMREAD_UNCHANGED);
        curr_frame->time_stamp_ = rgb_timestamps[i];
        Odometry::timer t0 = std::chrono::steady_clock::now();
        odometry->addFrame(curr_frame);
        Odometry::timer t1 = std::chrono::steady_clock::now();
        double time_cost = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
        LOG(ERROR) << "里程计处理一帧图像用时" << time_cost << "s.";
        time_cost_total += time_cost;
        Sophus::SE3d T_w_c = curr_frame->T_c_w_.inverse();
        // LOG(ERROR) << "R: " << T_w_c.rotationMatrix()
        //            << "\n t: " << T_w_c.translation();
        if ( odometry->state_ == myfrontend::Odometry::LOST )
            break;
        
        // show the map and the camera pose 
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                T_w_c.rotationMatrix()(0,0), T_w_c.rotationMatrix()(0,1), T_w_c.rotationMatrix()(0,2),
                T_w_c.rotationMatrix()(1,0), T_w_c.rotationMatrix()(1,1), T_w_c.rotationMatrix()(1,2),
                T_w_c.rotationMatrix()(2,0), T_w_c.rotationMatrix()(2,1), T_w_c.rotationMatrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                T_w_c.translation()(0,0), T_w_c.translation()(1,0), T_w_c.translation()(2,0)
            )
        );

        cv::imshow("image", curr_frame->color_);
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
    }
    LOG(ERROR) << "里程计处理一帧图像平均用时" 
               << time_cost_total/rgb_timestamps.size() << "s";
    return 0;
}