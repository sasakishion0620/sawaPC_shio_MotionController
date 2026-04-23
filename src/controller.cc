#include "controller.h"
#include "control_timer.h"
#include "thread_definition.h"
#include <cstdlib>
#include <cstdio>
#include <string>
#include <sys/stat.h>
#include <cerrno>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

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

#define Fx(n) robot.joints[(n)].data[mc::response][mc::Fx]
#define Fy(n) robot.joints[(n)].data[mc::response][mc::Fy]
#define Fz(n) robot.joints[(n)].data[mc::response][mc::Fz]
#define Mx(n) robot.joints[(n)].data[mc::response][mc::Mx]
#define My(n) robot.joints[(n)].data[mc::response][mc::My]
#define Mz(n) robot.joints[(n)].data[mc::response][mc::Mz]
#define Fx_offset(n) robot.joints[(n)].data[mc::offset][mc::Fx]
#define Fy_offset(n) robot.joints[(n)].data[mc::offset][mc::Fy]
#define Fz_offset(n) robot.joints[(n)].data[mc::offset][mc::Fz]
#define Mx_offset(n) robot.joints[(n)].data[mc::offset][mc::Mx]
#define My_offset(n) robot.joints[(n)].data[mc::offset][mc::My]
#define Mz_offset(n) robot.joints[(n)].data[mc::offset][mc::Mz]

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

  controller[mc::ForceOffsetInit] = [](robot_system &robot)
  {
    for (size_t i = 0; i < robot.joints.size(); ++i)
    {
      f_out(i) = 0.0;
    }

    if (robot.step() == 0)
    {
      robot.reset_force_offset_init();
    }

    if (!robot.force_sensor_connected || robot.joints.empty())
    {
      return;
    }

    if (robot.force_offset_samples < robot_system::force_offset_sample_count)
    {
      robot.force_offset_sum[0] += Fx(0);
      robot.force_offset_sum[1] += Fy(0);
      robot.force_offset_sum[2] += Fz(0);
      robot.force_offset_sum[3] += Mx(0);
      robot.force_offset_sum[4] += My(0);
      robot.force_offset_sum[5] += Mz(0);
      robot.force_offset_samples++;

      std::printf("force offset init: %zu / %zu\n",
        robot.force_offset_samples,
        robot_system::force_offset_sample_count);
      return;
    }

    if (!robot.force_offset_init_done)
    {
      const double sample_count = static_cast<double>(robot_system::force_offset_sample_count);
      Fx_offset(0) += robot.force_offset_sum[0] / sample_count;
      Fy_offset(0) += robot.force_offset_sum[1] / sample_count;
      Fz_offset(0) += robot.force_offset_sum[2] / sample_count;
      Mx_offset(0) += robot.force_offset_sum[3] / sample_count;
      My_offset(0) += robot.force_offset_sum[4] / sample_count;
      Mz_offset(0) += robot.force_offset_sum[5] / sample_count;
      robot.force_offset_init_done = true;

      std::printf(
        "force offset initialized: Fx=%lf Fy=%lf Fz=%lf Mx=%lf My=%lf Mz=%lf\n",
        Fx_offset(0), Fy_offset(0), Fz_offset(0),
        Mx_offset(0), My_offset(0), Mz_offset(0));

      robot.control_mode_request = mc::idle;
    }
  };

/*  // remote mode: inverse kinematics + PD control + DOB compensation
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

    // invert: small finger distance (grasp) ↁElarge x_d ↁElarge theta_cmd (robot closes)
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

    // invert: small finger distance (grasp) ↁElarge x_d ↁElarge theta_cmd (robot closes)
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
      if(f_vol(i) > 1.0){
        f_vol(i) = 1.0;
      }
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


  controller[mc::Angle_EMS] = [](robot_system &robot)
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

    // invert: small finger distance (grasp) ↁElarge x_d ↁElarge theta_cmd (robot closes)
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
    // for (size_t i = 0; i < robot.joints.size(); ++i)
    // {
    //   f_ref(i) = M(i) * (k_p(i) * (theta_cmd - x_res(i)) + k_v(i) * (dx_cmd - dx_res(i)));
    //   f_vol(i) = f_ref(i) + f_dis(i);
    //   if(f_vol(i) > 1.0){
    //     f_vol(i) = 1.0;
    //   }
    //   f_out(i) = f_vol(i);
    // }

    // EMS voltage calculation from f_dis(0)
    {
      // double f_threshold = robot.get_from_dict("ems_force_threshold");
      // double f_max       = robot.get_from_dict("ems_force_max");
      // double v_th        = robot.get_from_dict("ems_voltage_threshold");
      // double v_max       = robot.get_from_dict("ems_voltage_max");

      // double v_ems = 0.0;
      // if (f_dis(0) >= f_threshold)
      // {
      //   double denom = f_max - f_threshold;
      //   if (denom > 1e-9)
      //     v_ems = v_th + (v_max - v_th) / denom * (f_dis(0) - f_threshold);
      //   else
      //     v_ems = v_max;
      // }
      // if (v_ems < 0.0)   v_ems = 0.0;
      // if (v_ems > v_max) v_ems = v_max;

      // f_out(1) = v_ems;
      // robot.set_to_dict("da_ch1_voltage", v_ems);
      // robot.set_to_dict("ems_voltage", v_ems);

      
    }


    // --- EMS voltage calculation from theta_cmd (角度に基づく電圧計箁E ---
    {
      
      // 冁E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E��E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�玁E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E定義 (M_PIが使えなぁE�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E��E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�墁E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E��E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�のバックアチE�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E)
      #ifndef M_PI
      const double M_PI = 3.14159265358979323846;
      #endif

      // 角度の定義�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�度数法からラジアンへ変換�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E�E      const double deg2rad = M_PI / 180.0;
      double theta_min = 10.0 * deg2rad; // 10度
      double theta_max = 90.0 * deg2rad; // 90度
      
      double v_max = robot.get_from_dict("ems_voltage_threshold");
      double v_ems = robot.get_from_dict("ems_voltage_max");

      // センサーの現在値を取征E(単位�Eラジアンと想宁E
      double sensor_theta = x_res(0);

      // 10度(min)、E0度(max)の間で 0V、E.3V にスケーリング
      if (sensor_theta <= theta_min)
      {
        v_ems = 0.0;
      }
      else if (sensor_theta >= theta_max)
      {
        v_ems = v_max;
      }
      else
      {
        // 線形補間計箁E        v_ems = (sensor_theta- theta_min) / (theta_max - theta_min) * v_max;
      }

      // 出力に反映
      f_out(1) = v_ems;

      robot.set_to_dict("theta_res_deg", sensor_theta / deg2rad); // 確認用に度数法でも保孁E      robot.set_to_dict("da_ch1_voltage", v_ems);
      robot.set_to_dict("ems_voltage", v_ems);
    
    }

  };

*/

  controller[mc::PI_EMS] = [](robot_system &robot)
{
  static double integral = 0.0;
  static double V_in = 0.0;
  static double u_in = 0.0;
  static int count = 0;
  static FILE *fp = nullptr;
  static int time_count = 0;

  double uff = 0.0;

  constexpr int pi_update_interval_count = 10;
  constexpr double pi_update_dt = 0.001;

  const double K_ff = robot.get_from_dict("force_pi_K");
  const double Kp = robot.get_from_dict("force_pi_Kp");
  const long double Ki = robot.get_from_dict("force_pi_Ki");
  const double f_cmd = robot.get_from_dict("force_pi_f_cmd");
  const double uth = robot.get_from_dict("force_pi_uth");
  const double voltage_min = robot.get_from_dict("force_pi_voltage_min");
  const double voltage_max = robot.get_from_dict("force_pi_voltage_max");

  for (size_t i = 0; i < robot.joints.size(); ++i)
  {
    f_ref(i) = 0.0;
    f_out(i) = 0.0;
    f_vol(i) = 0.0;
  }

  if (robot.step() == 0)
  {
    integral = 0.0;
    V_in = 0.0;
    u_in = 0.0;
    count = 0;
    time_count = 0;
  }

  if (fp == nullptr)
  {
    const std::string data_dir = "../data";
    mkdir(data_dir.c_str(), 0755);

    boost::property_tree::ptree pt;
    boost::property_tree::read_json("../config/force_pi_control.json", pt);
    std::string record_file_name = pt.get<std::string>("record_file_name", "no_name.csv");

    if (record_file_name.size() < 4 || record_file_name.substr(record_file_name.size() - 4) != ".csv")
    {
      record_file_name += ".csv";
    }

    const std::string file_path = data_dir + "/" + record_file_name;
    fp = fopen(file_path.c_str(), "w");

    if (fp == nullptr)
    {
      printf("Error: Could not create file %s\n", file_path.c_str());
      exit(1);
    }

    fprintf(fp, "time,u_in,Vin,f_cmd,f_m,integral,uff\n");
    printf("New log file created: %s\n", file_path.c_str());
  }




  if (!robot.force_sensor_connected || robot.joints.empty())
  {
    integral = 0.0;
    V_in = 0.0;
    u_in = 0.0;
    count = 0;

    robot.set_to_dict("da_ch1_voltage", 0.0);
    robot.set_to_dict("ems_voltage", 0.0);
    exit(1);
  }

  const double f_m = Fz(0);

  robot.set_to_dict("force_pi_measured", f_m);

  count++;
  if (count >= pi_update_interval_count)
  {
    count = 0;

    const double e = f_cmd - f_m;
    integral += e * pi_update_dt;
    const double ufb = (Kp * e) + (Ki * integral);

    uff = uth;
    if (std::abs(K_ff) > 1e-9)
      uff = (f_cmd / K_ff) + uth;

    const double u = ufb + uth ;//+ uff;
    u_in = u;

    V_in = u_in;
    if (V_in < voltage_min) V_in = voltage_min;
    if (V_in > voltage_max) V_in = voltage_max;
  }

  robot.set_to_dict("force_pi_count", static_cast<double>(count));
  robot.set_to_dict("force_pi_v_in", V_in);
  robot.set_to_dict("da_ch1_voltage", V_in);
  robot.set_to_dict("ems_voltage", V_in);


  
  time_count++;

  
  if (time_count % 10 == 0)
  {
    if (fp != nullptr)
    {
      const double time = static_cast<double>(time_count) / 10000.0;
      fprintf(fp, "%.3f,%lf,%lf,%.2f,%lf,%lf,%lf\n", time, u_in, V_in, f_cmd, f_m, integral,uff);
      fflush(fp);
    }
  }

};

















  controller[mc::ModelDataRecord] = [](robot_system &robot)
  {
    static FILE *fp = nullptr;
    static long long time_count = 0;
    static double Vin = 0.0;

    //jsonが読めなかった時のデフォルト値，初期の値
    static std::string record_file_name = "default_test.csv";
    static double voltage_step = 0.2;
    static double voltage_max = 3.3;
    static long long step_count = 30000; // 10 kHz * 3 sec
    static long long record_count = 10;  // 10 kHz / 10 = 1 kHz
    static long long final_step_index = 16; // 0.0, 0.2, ..., 3.2


    //auto      → 型は自動で決めて,[&robot]  → 外のrobotを中でも使わせて
    auto output_zero = [&robot]()
    {
      for (size_t i = 0; i < robot.joints.size(); ++i)
      {
        f_ref(i) = 0.0;
        f_out(i) = 0.0;
        f_vol(i) = 0.0;
      }
      robot.set_to_dict("da_ch1_voltage", 0.0);
      robot.set_to_dict("ems_voltage", 0.0);
      robot.set_to_dict("voltage_step_v_in", 0.0);
    };
    
    //初回だけ実行する処理
    if (robot.step() == 0)
    {
      if (fp != nullptr)
      {
        fclose(fp);
        fp = nullptr;
      }

      time_count = 0;
      Vin = 0.0;
      output_zero();

      try
      {
        boost::property_tree::ptree pt;
        boost::property_tree::read_json("../config/model_data_record.json", pt);
        record_file_name = pt.get<std::string>("record_file_name", record_file_name);
        voltage_step = pt.get<double>("voltage_step", voltage_step);
        voltage_max = pt.get<double>("voltage_max", voltage_max);
        step_count = pt.get<long long>("step_count", step_count);
        record_count = pt.get<long long>("record_count", record_count);
        final_step_index = pt.get<long long>("final_step_index", final_step_index);//右側はなかったときの値
      }

      catch (...)
      {
        std::cerr << "[model_data_record] model_data_record.json not found, using defaults" << std::endl;
      }
      
      //設定値の補正
      if (record_file_name.size() < 4 || record_file_name.substr(record_file_name.size() - 4) != ".csv")
        record_file_name += ".csv";
      if (voltage_step <= 0.0) voltage_step = 0.2;
      if (voltage_max < 0.0) voltage_max = 0.0;
      if (step_count <= 0) step_count = 30000;
      if (record_count <= 0) record_count = 10;
      if (final_step_index < 0) final_step_index = 0;

      const std::string data_dir = "../data";

      mkdir(data_dir.c_str(), 0755);////c言語形式の文字列，権限
      const std::string file_path = data_dir + "/" + record_file_name;
      fp = fopen(file_path.c_str(), "w");

      if (fp == nullptr)
      {
        std::printf("[model_data_record] failed to create csv: %s\n", file_path.c_str());
        robot.control_mode_request = mc::idle;
        return;
      }

      std::fprintf(fp, "time,Vin,Force\n");
      std::printf("[model_data_record] started: csv=%s\n", file_path.c_str());
    }

    const long long finish_count = (final_step_index + 1) * step_count;

    if (time_count >= finish_count)
    {
      if (fp != nullptr)
      {
        fflush(fp);
        fclose(fp);
        fp = nullptr;
      }

      output_zero();
      std::printf("[model_data_record] finished. exiting.\n");
      exit(0);
    }

    if (time_count > 0 && time_count % step_count == 0)
    {
      Vin += voltage_step;

      if (Vin > voltage_max)
        Vin = voltage_max;
    }

    for (size_t i = 0; i < robot.joints.size(); ++i)
    {
      f_ref(i) = 0.0;
      f_out(i) = 0.0;
      f_vol(i) = 0.0;
    }

    const double measured_force = Fz(0);
    if (time_count % 10000 == 0)
    {
      const double time = static_cast<double>(time_count) / 10000.0;
      std::printf("time=%.3f, Vin=%.3f, Force=%.6f\n", time, Vin, measured_force);
    }

    robot.set_to_dict("da_ch1_voltage", Vin);
    robot.set_to_dict("ems_voltage", Vin);
    robot.set_to_dict("voltage_step_v_in", Vin);

    if (fp != nullptr && time_count % record_count == 0)
    {
      const double time = static_cast<double>(time_count) / 10000.0;
      std::fprintf(fp, "%.6f,%.6f,%.9f\n", time, Vin, measured_force);
    }

    time_count++;
  };

}

