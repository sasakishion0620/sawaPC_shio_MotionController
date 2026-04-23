#include "force_target_generator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;


//Vmin~Vmaxに抑える関数
double clamp_value(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}

//設定値のデフォルト値にリセット
void ForceTargetGenerator::load(const std::string &config_file, double fallback_value)
{
  type_ = Type::Constant;
  constant_value_ = fallback_value;
  offset_ = fallback_value;
  amplitude_ = 0.0;
  frequency_hz_ = 0.0;
  phase_rad_ = 0.0;
  min_ = 0.0;
  max_ = 0.0;
  clamp_enabled_ = false;
  low1_ = fallback_value;
  low2_ = fallback_value;
  high_ = fallback_value;
  low1_time_ = 0.0;
  rise_time_ = 0.0;
  high_time_ = 0.0;
  fall_time_ = 0.0;
  low2_time_ = 0.0;

  try
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(config_file, pt);

    //デフォルトはconstant

    const std::string type = pt.get<std::string>("type", "constant");

    if (type == "sin")
    {
      type_ = Type::Sin;
      offset_ = pt.get<double>("offset", fallback_value);
      amplitude_ = pt.get<double>("amplitude", 0.0);
      frequency_hz_ = pt.get<double>("frequency_hz", 0.0);
      phase_rad_ = pt.get<double>("phase_deg", 0.0) * kPi / 180.0;
      std::cout << "[force_target_generator] type=sin, offset=" << offset_
                << ", amplitude=" << amplitude_
                << ", frequency_hz=" << frequency_hz_ << std::endl;
      
      //get_optionalは値があれば取得，なければ空
      const auto min_opt = pt.get_optional<double>("min");
      const auto max_opt = pt.get_optional<double>("max");


      //&&はAかつB，つまり両方あるならば
      if (min_opt && max_opt)
      {
        min_ = *min_opt;
        max_ = *max_opt;
        clamp_enabled_ = min_ <= max_;/////結果はtrue or false
      }
    }

    //台形波
    else if (type == "trapezoid")
    {
      type_ = Type::Trapezoid;
      low1_ = pt.get<double>("low1", fallback_value);
      low2_ = pt.get<double>("low2", low1_);
      high_ = pt.get<double>("high", fallback_value);
      low1_time_ = std::max(0.0, pt.get<double>("low1_time", 0.0));
      rise_time_ = std::max(0.0, pt.get<double>("rise_time", 0.0));
      high_time_ = std::max(0.0, pt.get<double>("high_time", 0.0));
      fall_time_ = std::max(0.0, pt.get<double>("fall_time", 0.0));
      low2_time_ = std::max(0.0, pt.get<double>("low2_time", 0.0));
      std::cout << "[force_target_generator] type=trapezoid, low1=" << low1_
                << ", high=" << high_
                << ", low2=" << low2_ << std::endl;
    }


    //一定値
    else
    {
      if (type != "constant")
      {
        std::cerr << "[force_target_generator] unknown type: " << type
                  << ", using constant target" << std::endl;
      }
      type_ = Type::Constant;
      constant_value_ = pt.get<double>("value", fallback_value);
      std::cout << "[force_target_generator] type=constant, value=" << constant_value_ << std::endl;
    }
  }


  catch (const std::exception &e)
  {
    std::cerr << "[force_target_generator] " << config_file
              << " not loaded, using constant target: " << e.what() << std::endl;
  }
}



//まだ空白でいい
void ForceTargetGenerator::reset()
{
}



double ForceTargetGenerator::value(double time) const
{
  if (time < 0.0)
  {
    time = 0.0;
  }

  if (type_ == Type::Sin)
  {
    double target = offset_ + amplitude_ * std::sin((2.0 * kPi * frequency_hz_ * time) + phase_rad_);

    if (clamp_enabled_)
    {
      target = clamp_value(target, min_, max_);
    }
    return target;
  }


  if (type_ == Type::Trapezoid)
  {
    double t = time;

    //low維持区間
    if (t < low1_time_)
    {
      return low1_;
    }

    t -= low1_time_;


    //上昇区間
    if (t < rise_time_)
    {
      if (rise_time_ <= 0.0)
      {
        return high_;
      }
      return low1_ + (high_ - low1_) * (t / rise_time_);
    }


    //維持区間
    t -= rise_time_;
    if (t < high_time_)
    {
      return high_;
    }


    //下降区間
    t -= high_time_;
    if (t < fall_time_)
    {
      if (fall_time_ <= 0.0)
      {
        return low2_;
      }
      return high_ + (low2_ - high_) * (t / fall_time_);
    }


    //low維持区間
    t -= fall_time_;
    if (t < low2_time_)
    {
      return low2_;
    }

    return low2_;
  }

  return constant_value_;
}
