#pragma once
#include <iostream>
#include <math.h>

/// @brief 用于生成一个在给定范围内均匀分布的随机浮点数[lb,ub)
/// @param lb 下界
/// @param ub 上界
/// @return
inline double uniformRand(double lb, double ub) {
  return lb + ((double)std::rand() / (RAND_MAX + 1.0)) * (ub - lb);
}

/// @brief 高斯分布（正态分布）随机数
/// @param mean 均值
/// @param sigma 方差
/// @return
inline double gaussRand(double mean, double sigma) {
  double y, r2;
  do {
    double x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
    y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);

  return mean + sigma * y * sqrt(-2.0 * log(r2) / r2);
}