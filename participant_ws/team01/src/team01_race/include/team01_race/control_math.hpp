#pragma once
#include <algorithm>
#include <cmath>
#include <utility>
namespace team01_race {
inline std::pair<double,double> ackermann_joint_angles(
  double official_right_positive,double wheelbase,double track,double limit){
  const double center=std::clamp(-official_right_positive,-limit,limit);
  const double tangent=std::tan(center);
  const double left=std::atan2(wheelbase*tangent,wheelbase-0.5*track*tangent);
  const double right=std::atan2(wheelbase*tangent,wheelbase+0.5*track*tangent);
  return {std::clamp(left,-limit,limit),std::clamp(right,-limit,limit)};
}
}  // namespace team01_race
