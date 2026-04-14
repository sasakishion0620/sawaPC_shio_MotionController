#include <iostream>
#include "control_timer.h"
#include "thread_controller.h"
#include "leptrino.h"
#ifdef PCI_MODE
# include "contec_counter.h"
# include "contec_da.h"
#endif

using ll = long long int;

int main()
{
  ll sample_frequency = static_cast<ll>(atoi(mc::env::variable["sample_frequency"].c_str())); //Hz
  system_controller motion_control_system;
  motion_control_system.add_joint("../config/joints/4018_finger.json");
  motion_control_system.add_joint("../config/joints/4018_finger.json");
  motion_control_system.add_end();

#ifdef PCI_MODE
  contec_counter<double> counter1(0, 1, motion_control_system.get_robot(), "../config/contec_counter1.json");
  contec_da<double> da(0, 1, motion_control_system.get_robot(), "../config/contec_da1.json", 2);
  leptrino force_sensor;
  motion_control_system.add_reader(&counter1);
  motion_control_system.add_writer(&da);
  try
  {
    motion_control_system.init_leptrino(&force_sensor);
  }
  catch (const std::exception &e)
  {
    std::cerr << "[startup] failed to initialize force sensor: " << e.what() << std::endl;
    return 1;
  }
#else
  motion_control_system.get_robot()->force_sensor_enabled = false;
  motion_control_system.get_robot()->force_sensor_connected = false;
#endif

  // setting thread controller
  thread_controller motion_control_threads(sample_frequency, &motion_control_system);

  motion_control_threads.thread_registeration();
  motion_control_threads.start_timer();
  return 0;
}
