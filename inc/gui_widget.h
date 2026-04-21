#ifndef TELEOPHAND_GUI_WIDGET_H
#define TELEOPHAND_GUI_WIDGET_H
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "robot_system.h"
#include "implot.h"
#include "enum_helper.h"
#include "thread_definition.h"

namespace mc {
  struct scrolling_buffer
  {
    int max_size;
    int offset;
    ImVector<ImVec2> data;
    scrolling_buffer(int max_size = 2000) {
      this->max_size = max_size;
      offset  = 0;
      data.reserve(max_size);
    }
    void add_point(float x, float y) {
      if (data.size() < max_size)
        data.push_back(ImVec2(x,y));
      else {
        data[offset] = ImVec2(x,y);
        offset =  (offset + 1) % max_size;
      }
    }
    void erase() {
      if (data.size() > 0) {
        data.shrink(0);
        offset  = 0;
      }
    }
  };

  // shared gui time for synchronized plot axes
  static inline float gui_time = 0.0f;

  class widget
  {
  public:
    static void display_status(robot_system *robot_system_ptr)
    {
      if (robot_system_ptr->joints.empty())
      {
        ImGui::Text("Force sensor data unavailable");
        return;
      }

      const bool force_sensor_connected = robot_system_ptr->force_sensor_enabled && robot_system_ptr->force_sensor_connected;
      const ImVec4 value_color = force_sensor_connected
        ? ImVec4(0.90f, 0.95f, 0.35f, 1.0f)
        : ImVec4(0.60f, 0.60f, 0.60f, 1.0f);
      const char *labels[6] = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
      const double values[6] = {
        robot_system_ptr->joints.at(0).data[mc::response][mc::Fx],
        robot_system_ptr->joints.at(0).data[mc::response][mc::Fy],
        robot_system_ptr->joints.at(0).data[mc::response][mc::Fz],
        robot_system_ptr->joints.at(0).data[mc::response][mc::Mx],
        robot_system_ptr->joints.at(0).data[mc::response][mc::My],
        robot_system_ptr->joints.at(0).data[mc::response][mc::Mz]
      };

      ImGui::Columns(6, "force_sensor_values");
      for (int i = 0; i < 6; ++i)
      {
        ImGui::Text("%s", labels[i]);
        ImGui::SetWindowFontScale(1.8f);
        ImGui::TextColored(value_color, "%+.3f", values[i]);
        ImGui::SetWindowFontScale(1.0f);
        if (i < 5)
          ImGui::NextColumn();
      }
      ImGui::Columns(1);
    }

    static void control_mode_selection(robot_system *robot_system_ptr)
    {
      ImGui::Columns(mc::control_mode_size, "control mode");

      for (int i = 0; i < mc::control_mode_size; ++i)
      {
        const auto mode = static_cast<mc::control_mode>(i);
        std::string mode_name = mc::enum_helper::name(mode);
        if(ImGui::Button(mode_name.c_str(), ImVec2(-1.0f, 0.0f)))
        {
          robot_system_ptr->control_mode_request = static_cast<mc::control_mode>(i);
          std::cout << "mode [ " << mode_name << "] is seleted" << std::endl ;
        }
        ImGui::NextColumn();
      }
    }

#define Fx(n) robot_system_ptr->joints[(n)].data[mc::response][mc::Fx]
#define Fy(n) robot_system_ptr->joints[(n)].data[mc::response][mc::Fy]
#define Fz(n) robot_system_ptr->joints[(n)].data[mc::response][mc::Fz]
#define Mx(n) robot_system_ptr->joints[(n)].data[mc::response][mc::Mx]
#define My(n) robot_system_ptr->joints[(n)].data[mc::response][mc::My]
#define Mz(n) robot_system_ptr->joints[(n)].data[mc::response][mc::Mz]
    static void plot_state(robot_system *robot_system_ptr)
    {
      if (robot_system_ptr->joints.empty())
      {
        ImGui::Text("No signal to plot");
        return;
      }

      static std::vector<scrolling_buffer> sdata_vec(7);
      static float history = 5.0f;

      gui_time += ImGui::GetIO().DeltaTime;
      float t = gui_time;
      const bool force_sensor_connected = robot_system_ptr->force_sensor_enabled && robot_system_ptr->force_sensor_connected;
      const bool is_force_pi_control = robot_system_ptr->get_control_mode() == mc::PI_EMS;
      sdata_vec[0].add_point(t, static_cast<float>(Fx(0)));
      sdata_vec[1].add_point(t, static_cast<float>(Fy(0)));
      sdata_vec[2].add_point(t, static_cast<float>(Fz(0)));
      sdata_vec[3].add_point(t, static_cast<float>(Mx(0)));
      sdata_vec[4].add_point(t, static_cast<float>(My(0)));
      sdata_vec[5].add_point(t, static_cast<float>(Mz(0)));
      sdata_vec[6].add_point(t, static_cast<float>(robot_system_ptr->get_from_dict("force_pi_f_cmd")));

      ImPlot::SetNextPlotLimitsX(t - history, t + history, ImGuiCond_Always);
      if (sdata_vec[0].data.size() > 0 && ImPlot::BeginPlot("Force", "time [sec]", "force", ImVec2(-1, 320)))
      {
        ImPlot::PlotLine("Fx", &sdata_vec[0].data[0].x, &sdata_vec[0].data[0].y, sdata_vec[0].data.size(), sdata_vec[0].offset, 2*sizeof(float));
        ImPlot::PlotLine("Fy", &sdata_vec[1].data[0].x, &sdata_vec[1].data[0].y, sdata_vec[1].data.size(), sdata_vec[1].offset, 2*sizeof(float));
        if (force_sensor_connected)
          ImPlot::PlotLine("Fz", &sdata_vec[2].data[0].x, &sdata_vec[2].data[0].y, sdata_vec[2].data.size(), sdata_vec[2].offset, 2*sizeof(float));
        if (is_force_pi_control)
          ImPlot::PlotLine("f_cmd", &sdata_vec[6].data[0].x, &sdata_vec[6].data[0].y, sdata_vec[6].data.size(), sdata_vec[6].offset, 2*sizeof(float));
        ImPlot::EndPlot();
      }

      ImPlot::SetNextPlotLimitsX(t - history, t + history, ImGuiCond_Always);
      if (sdata_vec[3].data.size() > 0 && ImPlot::BeginPlot("Moment", "time [sec]", "moment", ImVec2(-1, 320)))
      {
        ImPlot::PlotLine("Mx", &sdata_vec[3].data[0].x, &sdata_vec[3].data[0].y, sdata_vec[3].data.size(), sdata_vec[3].offset, 2*sizeof(float));
        ImPlot::PlotLine("My", &sdata_vec[4].data[0].x, &sdata_vec[4].data[0].y, sdata_vec[4].data.size(), sdata_vec[4].offset, 2*sizeof(float));
        if (force_sensor_connected)
          ImPlot::PlotLine("Mz", &sdata_vec[5].data[0].x, &sdata_vec[5].data[0].y, sdata_vec[5].data.size(), sdata_vec[5].offset, 2*sizeof(float));
        ImPlot::EndPlot();
      }
    }
#undef Fx
#undef Fy
#undef Fz
#undef Mx
#undef My
#undef Mz
  };
} // namespace mc
#endif //TELEOPHAND_GUI_WIDGET_H
