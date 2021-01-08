#pragma once
#include <limits>
#include <vector>
class Box2d {
 public:
  Box2d() = default;
  Box2d(const std::vector<double> &center, const double &heading, const double &length_,  const double &width_);
  void GetAllCorners(std::vector<std::vector<double>> * const corners) const;
  void InitCorners();
  
 private:
  std::vector<double> center_;
  double length_ = 4.68;
  double width_ = 1.93;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;

  std::vector<std::vector<double>> corners_;
};
