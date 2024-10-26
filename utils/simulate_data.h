/*
    @brief 本文件放置模拟数据生成相关的类
 */
#pragma once
#include "pose.hpp"
#include "random.hpp"
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace utils {
/// @brief 本类BundleAdjust的模拟数据。模型为相机，空间中存在模拟3D点。
class CamBASimData {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CamBASimData(size_t pose_nums, size_t pt_nums, double pixel_noise,
               double focal_length, double cx, double cy, double width,
               double height)
      : pose_nums_(pose_nums), pt_nums_(pt_nums), pixel_noise_(pixel_noise),
        focal_length_(focal_length), cx_(cx), cy_(cy), width_(width),
        height_(height) {}
  void generateData();

private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector3d> noise_points_;
  std::vector<pose::QuanPose<double>> poses_;
  size_t pose_nums_;
  size_t pt_nums_;
  double pixel_noise_;
  double focal_length_;
  double cx_;
  double width_;
  double height_;
  double cy_;
};
}; // namespace utils