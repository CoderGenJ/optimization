#pragma once
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace pose {
template <typename T> class QuanPose {
public:
  QuanPose(const Eigen::Matrix<T, 3, 1> &pos,
           const Eigen::Quaternion<T> &orient)
      : position(pos), orientation(orient) {}

  QuanPose() {
    position.setZero();
    orientation.setIdentity();
  }

  const Eigen::Matrix<T, 3, 1> &getPosition() const { return position; }

  const Eigen::Quaternion<T> &getOrientation() const { return orientation; }

  void print() const {
    std::cout << "Position: [" << position.transpose() << "]\n";
    std::cout << "Orientation: [" << orientation.coeffs().transpose()
              << "]\n"; // 四元数系数
  }

  QuanPose &operator=(const QuanPose &other) {
    if (this != &other) {
      position = other.position;
      orientation = other.orientation;
    }
    return *this;
  }

private:
  Eigen::Matrix<T, 3, 1> position;
  Eigen::Quaternion<T> orientation;
};

} // namespace pose
