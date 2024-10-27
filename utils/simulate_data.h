/*
    @brief 本文件放置模拟数据生成相关的类
 */
#pragma once
#include "pose.hpp"
#include "random.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <list>
#include <vector>

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
  ~CamBASimData() {}
  CamBASimData() = default;
  //禁止复制
  CamBASimData(const CamBASimData &) = delete;
  CamBASimData &operator=(const CamBASimData &) = delete;

private:
  //随机生成的空间3D点
  std::vector<Eigen::Vector3d> points_;
  //随机生成的空间3D点增加了一些噪声在每个维度
  std::vector<Eigen::Vector3d> noise_points_;
  //共视超过2个视角的map点被计为可用点
  std::vector<bool> valid_points_;
  //按照一定规律生成的pose
  std::vector<pose::QuanPose<double>> poses_;
  //每个视角看到的map点的id
  std::vector<std::list<size_t>> marker_id_in_pose_;
  // map点的实际测量值，真值+随机高斯噪声
  std::vector<std::list<std::pair<double, double>>> meas_in_pose_;

  //随机生成的pose个数
  size_t pose_nums_;
  //空间中随机生成的点的个数
  size_t pt_nums_;

  double pixel_noise_;
  //相机参数
  double focal_length_;
  double cx_;
  double cy_;
  //图片的宽高
  double width_;
  double height_;
};
}; // namespace utils