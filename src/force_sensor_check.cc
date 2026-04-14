#include <iostream>
#include "leptrino.h"

int main()
{
  leptrino force_sensor;
  if (!force_sensor.init())
  {
    std::cerr << "[force_sensor_check] failed to open or communicate with force sensor" << std::endl;
    return 1;
  }

  std::cout << "[force_sensor_check] force sensor COM check passed" << std::endl;
  force_sensor.App_Close();
  return 0;
}
