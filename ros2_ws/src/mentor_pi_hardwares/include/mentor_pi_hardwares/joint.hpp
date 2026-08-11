#ifndef MENTOR_PI_HARDWARES__JOINT_HPP_
#define MENTOR_PI_HARDWARES__JOINT_HPP_

#include <string>

namespace mentor_pi {

struct JointValue {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint {
  explicit Joint(const std::string& name) : joint_name(name) {}
  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;
};

}  // namespace mentor_pi

#endif  // MENTOR_PI_HARDWARES__JOINT_HPP_
