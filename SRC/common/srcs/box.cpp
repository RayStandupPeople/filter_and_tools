/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "../libs/box.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <iostream>

Box2d::Box2d(const std::vector<double> &center, const double &heading)
    : center_(center),
      half_length_(length_ / 2.0),
      half_width_(width_ / 2.0),
      heading_(heading),
      cos_heading_(std::cos(heading)),
      sin_heading_(std::sin(heading)) {
  InitCorners();
}

void Box2d::InitCorners() {
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners_.clear();
  corners_.emplace_back(std::vector<double>{center_[0] + dx1 + dx2, center_[1] + dy1 + dy2});
  corners_.emplace_back(std::vector<double>{center_[0] + dx1 - dx2, center_[1] + dy1 - dy2});
  corners_.emplace_back(std::vector<double>{center_[0] - dx1 - dx2, center_[1] - dy1 - dy2});
  corners_.emplace_back(std::vector<double>{center_[0] - dx1 + dx2, center_[1] - dy1 + dy2});
}

void Box2d::GetAllCorners(std::vector<std::vector<double>> * const corners) const {
  if (corners == nullptr) {
    return;
  }
  *corners = corners_;
}

