#ifndef MOTIONCONTROL_ROBOT_SYSTEM_H
#define MOTIONCONTROL_ROBOT_SYSTEM_H
#include <iostream>
#include <vector>
#include <unordered_map>
#include <array>
#include "joint.hpp"
#include "mode_definition.h"

struct robot_system
{
public:
  // data
  std::vector<joint<double>> joints;
  std::vector<long long int> task_takt_times;     // duration for excuting each thread
  std::vector<long long int> timer_sampling_times;// duration between each timer's sampling call
  mc::control_mode control_mode_request;
  bool is_recording = false;
  bool force_sensor_enabled = false;
  bool force_sensor_connected = false;
  static constexpr size_t force_offset_sample_count = 1000;
  std::array<double, 6> force_offset_sum = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  size_t force_offset_samples = 0;
  bool force_offset_init_done = false;

  // method
  robot_system()
  {
    control_mode_request = mc::idle;
    control_mode_state = control_mode_request;
    control_step_ = 0;
    std::cout << "constructor robot_system" << std::endl;
  }
  // data
  bool is_control_mode_changed(){ return control_mode_request != control_mode_state; }
  void update_control_mode(){control_mode_state = control_mode_request;}
  mc::control_mode get_control_mode(){ return control_mode_state; }

  // methods
  void add_joint(const char* filename){ joints.push_back(filename); }
  void add_end()
  {
    std::cout << "robot setting has been finished" << std::endl;
  }
  void increase_control_step(){ control_step_++;}
  void reset_control_step(){ control_step_ = 0;}
  long long int step(){ return control_step_; }
  void reset_force_offset_init()
  {
    force_offset_sum.fill(0.0);
    force_offset_samples = 0;
    force_offset_init_done = false;
  }
  void set_to_dict(const char *key, double value){general_dictionary_[key] = value;}
  double get_from_dict(const char *key)
  {
    if (general_dictionary_.find(key) != general_dictionary_.end())
      return general_dictionary_[key];
    else
      return 0.0;
  }
private:
  mc::control_mode control_mode_state;
  long long int control_step_;
  std::unordered_map<std::string, double> general_dictionary_;
};
#endif //MOTIONCONTROL_ROBOT_SYSTEM_H
