#include "simulate_data.h"

namespace utils {
void CamBASimData::generateData() {
  // generate true poses
  for (size_t i = 0; i < pose_nums_; i++) {
    Eigen::Matrix<double, 3, 1> pos = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Quaternion<double> orient = Eigen::Quaternion<double>::Identity();
    pos.x() = i * 0.04 - 1.0;
    pose::QuanPose<double> pose(pos, orient);
    poses_.emplace_back(pose);
  }

  // generate true points
  for (size_t i = 0; i < pt_nums_; i++) {
    Eigen::Vector3d pt{(uniformRand(0.0, 1.0) - 0.5) * 3,
                       uniformRand(0.0, 1.0) - 0.5,
                       uniformRand(0.0, 1.0) + 3.0};
    points_.emplace_back(pt);
  }

  // generate noise points
  for (size_t i = 0; i < points_.size(); ++i) {
    Eigen::Vector3d pt_noise;
    pt_noise.x() = points_.at(i).x() + gaussRand(0.0, pixel_noise_);
    pt_noise.y() = points_.at(i).y() + gaussRand(0.0, pixel_noise_);
    pt_noise.z() = points_.at(i).z() + gaussRand(0.0, pixel_noise_);
    noise_points_.emplace_back(pt_noise);
  }
  // 
}

} // namespace utils