#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

int main() {
  // 生成样本数据并增加高斯噪声
  double a_true = 1.0;
  double b_true = 2.0;
  double c_true = 1.0;

  double a_esti = 1.3;
  double b_esti = 2.3;
  double c_esti = 1.3;

  double w_sigma = 1.0;
  double w_sigma_inv = 1.0 / w_sigma;
  int N = 100;
  std::vector<int> y_data, x_data;
  cv::RNG rng;
  for (int i = 0; i < N; ++i) {
    double x = i / 100;
    x_data.push_back(x);
    y_data.push_back(std::exp(a_true * x * x + b_true * x + c_true) +
                     rng.gaussian(w_sigma * w_sigma));
  }
  // 迭代求解
  int iteration_max = 100;

  double epsilon = 1e-9;
  for (int j = 0; j < iteration_max; ++j) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    double cost = 0;
    for (int i = 0; i < N; ++i) {
      double x = x_data.at(i);
      double y = y_data.at(i);
      Eigen::Vector3d J = Eigen::Vector3d::Zero();
      double exp_value = std::exp(a_esti * x * x + b_esti * x + c_esti);
      J[0] = (-1) * x * x * exp_value; // e/a
      J[1] = (-1) * x * exp_value;     // e/b
      J[2] = (-1) * exp_value;         // e/c
      H += w_sigma_inv * w_sigma_inv * J * J.transpose();
      double error = y - exp(a_esti * x * x + b_esti * x + c_esti);
      b += w_sigma_inv * w_sigma_inv * -1.0 * error * J;

      cost += error * error;
    }
    //解 Hx=b
    Eigen::Vector3d d_esti = H.ldlt().solve(b);

    //更新估计值
    if (d_esti.norm() < epsilon) {
      std::cout << "更新足够小，收敛完成。" << std::endl;
      break;
    }
    std::cout << "cost:" << cost << std::endl;
    a_esti += d_esti[0];
    b_esti += d_esti[1];
    c_esti += d_esti[2];
  }
  std::cout << "estimater a:" << a_esti << " b:" << b_esti << " c:" << c_esti
            << std::endl;

  return 0;
}