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
  // 标记可用的点,被至少两个相机视角看到的map点
  Eigen::Matrix3d K;
  K << focal_length_, 0.0, cx_, 0.0, focal_length_, cy_, 0.0, 0.0, 1.0;
  for (size_t i = 0; i < pt_nums_; i++) {
    int num_obs = 0;
    for (const auto &pose : poses_) {
      Eigen::Vector3d pc =
          pose.getOrientation() * points_.at(i) + pose.getPosition();
      if (pc(2) <= std::numeric_limits<double>::epsilon()) {
        continue;
      }
      pc = pc / pc(2);
      Eigen::Vector3d z = K * pc;
      //判断投影点是否在图像范围内,通过这种方式表示是否视野中。
      if (z(0) >= 0 && z(0) < width_ && z(1) >= 0 && z(1) < height_) {
        num_obs++;
      }
    }
    if (num_obs >= 2) {
      valid_points_[i] = true;
    } else {
      valid_points_[i] = false;
    }
  }

  //记录每个视角看到的mark点
  for (size_t i = 0; i < poses_.size(); ++i) {
    for (size_t j = 0; j < pt_nums_; ++j) {
      Eigen::Vector3d pc =
          poses_[i].getOrientation() * points_[j] + poses_[i].getPosition();
      //判断Z是否接近0
      if (pc(2) <= std::numeric_limits<double>::epsilon())
        continue;
      pc = pc / pc(2);
      Eigen::Vector3d z = K * pc;
      if (z(0) >= 0 && z(0) < width_ && z(1) >= 0 && z(1) < height_) {
        marker_id_in_pose_[i].push_back(j);
        std::pair<double, double> noise_z;
        //测量=真值+噪音
        noise_z.first = z(0) + gaussRand(0.0, pixel_noise_);
        noise_z.second = z(1) + gaussRand(0.0, pixel_noise_);
        meas_in_pose_[i].push_back(noise_z);
      }
    }
  }
}

} // namespace utils