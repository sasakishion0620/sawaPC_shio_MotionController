#include "controller.h"
#include "control_timer.h"
#include "thread_definition.h"

#define x_res(n) robot.joints[(n)].data[mc::response][mc::x]
#define dx_res(n) robot.joints[(n)].data[mc::response][mc::dx]
#define ddx_res(n) robot.joints[(n)].data[mc::response][mc::ddx]
#define f_dis(n) robot.joints[(n)].data[mc::response][mc::f_dis]
#define f_ref(n) robot.joints[(n)].data[mc::reference][mc::f]
#define f_out(n) robot.joints[(n)].data[mc::output][mc::f]
#define f_vol(n) robot.joints[(n)].data[mc::output][mc::f_dis]
#define k_p(n) robot.joints[(n)].parameter[mc::k_p]
#define k_v(n) robot.joints[(n)].parameter[mc::k_v]
#define k_f(n) robot.joints[(n)].parameter[mc::k_f]
#define M(n) robot.joints[(n)].parameter[mc::mass]
#define D(n) robot.joints[(n)].parameter[mc::damper]
#define K(n) robot.joints[(n)].parameter[mc::spring]
#define param(n) robot.joints[(n)].parameter
void mc::control::default_controller(robot_system &robot)
{
  for (size_t i = 0; i < robot.joints.size(); ++i)
  {
    f_out(i) = 0.0;
  }
}

void mc::control::register_controller()
{
  controller[mc::idle] = [](robot_system &robot)
  {
    for (size_t i = 0; i < robot.joints.size(); ++i)
    {
      f_out(i) = 0.0;
    }
  };

  controller[mc::DA_check] = [](robot_system &robot)
  {
    for (size_t i = 0; i < robot.joints.size(); ++i)
    {
      f_out(i) = 0.3;
    }
  };

  // remote mode: inverse kinematics + PD control + DOB compensation
  controller[mc::remote] = [](robot_system &robot)
  {
    static double buf_cmd = 0.0;
    //Kita add
    static double dx_buf = 0.0;
    static double dx_cmd = 0.0;

    double r = robot.get_from_dict("link_length");    // link length [m]
    double g_cmd = robot.get_from_dict("g_cmd");       // command filter gain
    double max_dist = robot.get_from_dict("max_distance_mm"); // max distance [mm]
    double dt = robot.get_from_dict("dt");  // sampling period (set from system.json)

    // mode switch reset: initialize buf_cmd to current position
    if (robot.step() == 0)
    {
      buf_cmd = x_res(0);
    }

    // get distance from finger-tracker
    double distance_mm = robot.get_from_dict("distance_mm");

    // clamp distance
    if (distance_mm < 0.0) distance_mm = 0.0;
    if (distance_mm > max_dist) distance_mm = max_dist;

    // invert: small finger distance (grasp) → large x_d → large theta_cmd (robot closes)
    double x_d = (max_dist - distance_mm) / 1000.0;

    // clamp to valid range for inverse kinematics (0 to 2r)
    double max_x = 2.0 * r;
    if (x_d > max_x) x_d = max_x;
    if (x_d < 0.0) x_d = 0.0;

    // inverse kinematics: theta_cmd = acos(1 - x_d^2 / (2*r^2))
    double cos_arg = 1.0 - (x_d * x_d) / (2.0 * r * r);
    // clamp cos argument to [-1, 1]
    if (cos_arg > 1.0) cos_arg = 1.0;
    if (cos_arg < -1.0) cos_arg = -1.0;
    double theta_cmd_raw = acos(cos_arg);

    // command filter: first-order low-pass
    buf_cmd += dt * g_cmd * (theta_cmd_raw - buf_cmd);
    double theta_cmd = buf_cmd;

    //Kita add
    dx_cmd = g_cmd * (theta_cmd - dx_buf);
    dx_buf += dt * dx_cmd;

    // store theta_cmd for recording and GUI
    robot.set_to_dict("theta_cmd", theta_cmd);

    // PD control + DOB compensation for each joint
    for (size_t i = 0; i < robot.joints.size(); ++i)
    {
      f_ref(i) = M(i) * (k_p(i) * (theta_cmd - x_res(i)) + k_v(i) * (dx_cmd - dx_res(i)));
      f_out(i) = f_ref(i) + f_dis(i);
    }
  };

  // Record mode uses the same control law as remote
  controller[mc::Record] = controller[mc::remote];

  // Bilateral mode: remote control + EMS force feedback
  controller[mc::Bilateral] = [](robot_system &robot)
  {
    static double buf_cmd = 0.0;
    //Kita add
    static double dx_buf = 0.0;
    static double dx_cmd = 0.0;
    double r = robot.get_from_dict("link_length");    // link length [m]
    double g_cmd = robot.get_from_dict("g_cmd");       // command filter gain
    double max_dist = robot.get_from_dict("max_distance_mm"); // max distance [mm]
    double dt = robot.get_from_dict("dt");  // sampling period (set from system.json)

    // mode switch reset: initialize buf_cmd to current position
    if (robot.step() == 0)
    {
      buf_cmd = x_res(0);
    }

    // get distance from finger-tracker
    double distance_mm = robot.get_from_dict("distance_mm");

    // clamp distance
    if (distance_mm < 0.0) distance_mm = 0.0;
    if (distance_mm > max_dist) distance_mm = max_dist;

    // invert: small finger distance (grasp) → large x_d → large theta_cmd (robot closes)
    double x_d = (max_dist - distance_mm) / 1000.0;

    // clamp to valid range for inverse kinematics (0 to 2r)
    double max_x = 2.0 * r;
    if (x_d > max_x) x_d = max_x;
    if (x_d < 0.0) x_d = 0.0;

    // inverse kinematics: theta_cmd = acos(1 - x_d^2 / (2*r^2))
    double cos_arg = 1.0 - (x_d * x_d) / (2.0 * r * r);
    // clamp cos argument to [-1, 1]
    if (cos_arg > 1.0) cos_arg = 1.0;
    if (cos_arg < -1.0) cos_arg = -1.0;
    double theta_cmd_raw = acos(cos_arg);

    // command filter: first-order low-pass
    buf_cmd += dt * g_cmd * (theta_cmd_raw - buf_cmd);
    double theta_cmd = buf_cmd;

    //Kita add
    dx_cmd = g_cmd * (theta_cmd - dx_buf);
    dx_buf += dt * dx_cmd;

    // store theta_cmd for recording and GUI
    robot.set_to_dict("theta_cmd", theta_cmd);

    // PD control + DOB compensation for each joint
    for (size_t i = 0; i < robot.joints.size(); ++i)
    {
      f_ref(i) = M(i) * (k_p(i) * (theta_cmd - x_res(i)) + k_v(i) * (dx_cmd - dx_res(i)));
      f_vol(i) = f_ref(i) + f_dis(i);
      f_out(i) = f_vol(i);
    }

    // EMS voltage calculation from f_dis(0)
    {
      double f_threshold = robot.get_from_dict("ems_force_threshold");
      double f_max       = robot.get_from_dict("ems_force_max");
      double v_th        = robot.get_from_dict("ems_voltage_threshold");
      double v_max       = robot.get_from_dict("ems_voltage_max");

      double v_ems = 0.0;
      if (f_dis(0) >= f_threshold)
      {
        double denom = f_max - f_threshold;
        if (denom > 1e-9)
          v_ems = v_th + (v_max - v_th) / denom * (f_dis(0) - f_threshold);
        else
          v_ems = v_max;
      }
      if (v_ems < 0.0)   v_ems = 0.0;
      if (v_ems > v_max) v_ems = v_max;

      f_out(1) = v_ems;
      robot.set_to_dict("da_ch1_voltage", v_ems);
      robot.set_to_dict("ems_voltage", v_ems);
    }

  };
}
