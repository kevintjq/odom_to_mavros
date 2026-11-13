#ifndef ODOM_TO_MAVROS_EXTRINSIC_VALIDATION_H
#define ODOM_TO_MAVROS_EXTRINSIC_VALIDATION_H
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

inline std::array<double, 6> validatedExtrinsic(std::array<double, 6> values,
                                              const std::string& unit)
{
  const char* names[] = {"translation/x", "translation/y", "translation/z",
                         "rotation/roll", "rotation/pitch", "rotation/yaw"};
  for (unsigned i = 0; i < values.size(); ++i)
    if (!std::isfinite(values[i]))
      throw std::runtime_error(std::string("Non-finite extrinsic/") + names[i]);
  if (unit == "deg" || unit == "degree" || unit == "degrees") {
    for (unsigned i = 3; i < values.size(); ++i)
      values[i] *= std::acos(-1.0) / 180.0;
  } else if (unit != "rad" && unit != "radian" && unit != "radians") {
    throw std::runtime_error("Unknown extrinsic/rotation/unit: " + unit);
  }
  return values;
}
#endif
