#ifndef MOTIONCONTOL_AD_READER_H
#define MOTIONCONTOL_AD_READER_H

#include <iostream>
#include <vector>
#include "robot_system.h"

template<typename T>
class adreader
{
public:
  adreader(
    size_t start_channel,
    size_t number_of_channel,
    robot_system *robot,
    mc::state adread_state = mc::ad_voltage)
  : start_channel_(start_channel),
    number_of_channel_(number_of_channel),
    adread_data_ptr_(number_of_channel_),
    adread_raw_ptr_(number_of_channel_),
    adread_state_(adread_state)
  {
    register_adread_data(robot);
    device_id_++;
    std::cout << "ad reader constructor" << std::endl;
  }

  virtual int open() = 0;
  virtual int adread()
  {
    scan();
    return number_of_channel_;
  }
  virtual int scan() = 0;
  virtual void reset() = 0;
  virtual void initialize() = 0;
  virtual void close() = 0;
  T *channel_ptr(size_t i){ return adread_data_ptr_[i]; }
  T *raw_channel_ptr(size_t i){ return adread_raw_ptr_[i]; }

protected:
  size_t start_channel_;
  size_t number_of_channel_;
  mc::state adread_state_;

private:
  static int device_id_;
  std::vector<T*> adread_data_ptr_;
  std::vector<T*> adread_raw_ptr_;

  void register_adread_data(robot_system *robot)
  {
    if (robot->joints.size() < start_channel_ + number_of_channel_)
    {
      std::cerr << "Warning:" << std::endl;
      std::cerr << "\t You try to register unexist joint." << std::endl;
      std::cerr << "\t number_of_channel is too large." << std::endl;
      std::cerr << "\t device id: " << device_id_ << std::endl;
      exit(1);
    }

    for (size_t i = 0; i < number_of_channel_; ++i)
    {
      adread_data_ptr_.at(i) =
        &(robot->joints.at(start_channel_ + i).data[mc::response][adread_state_]);
      adread_raw_ptr_.at(i) =
        &(robot->joints.at(start_channel_ + i).data[mc::response][mc::ad_raw_count]);
    }
  }
};

template<typename T>
int adreader<T>::device_id_ = 0;

#endif // MOTIONCONTOL_AD_READER_H
