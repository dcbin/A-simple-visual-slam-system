/**
 * @file common.h
 * @brief Common header file for the frontend
 * @date 2025-02-18
 * @author Chengbin Duan
 * @ref Gaoxiang slambook2
 */

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <glog/logging.h>
#include <sophus/se3.hpp>

#include <vector>
#include <list>
#include <map>
#include <unordered_map>

using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d;

#endif // COMMON_INCLUDE_H