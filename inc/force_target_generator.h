#ifndef MOTIONCONTROL_FORCE_TARGET_GENERATOR_H
#define MOTIONCONTROL_FORCE_TARGET_GENERATOR_H

#include <string>

class ForceTargetGenerator
{
public:
  void load(const std::string &config_file, double fallback_value);
  void reset();
  double value(double time) const;

private:
  enum class Type
  {
    Constant,
    Sin,
    Trapezoid
  };

  Type type_ = Type::Constant;
  double constant_value_ = 0.0;

  double offset_ = 0.0;
  double amplitude_ = 0.0;
  double frequency_hz_ = 0.0;
  double phase_rad_ = 0.0;
  double min_ = 0.0;
  double max_ = 0.0;
  bool clamp_enabled_ = false;

  double low1_ = 0.0;
  double low2_ = 0.0;
  double high_ = 0.0;
  double low1_time_ = 0.0;
  double rise_time_ = 0.0;
  double high_time_ = 0.0;
  double fall_time_ = 0.0;
  double low2_time_ = 0.0;
};

#endif // MOTIONCONTROL_FORCE_TARGET_GENERATOR_H
