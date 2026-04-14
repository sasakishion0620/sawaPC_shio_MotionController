omoya@tomoya-MS-7C56:~/sawaPC_shio_MotionController/build$ cmake ..
-- The C compiler identification is GNU 9.4.0
-- The CXX compiler identification is GNU 9.4.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: /home/tomoya/sawaPC_shio_MotionController/build
tomoya@tomoya-MS-7C56:~/sawaPC_shio_MotionController/build$ make -j
Scanning dependencies of target GUI
Scanning dependencies of target force_sensor_check
[ 4%] Building CXX object CMakeFiles/force_sensor_check.dir/src/leptrino.cc.o
[ 8%] Building CXX object CMakeFiles/force_sensor_check.dir/src/force_sensor_check.cc.o
[ 20%] Building C object lib/CMakeFiles/GUI.dir/imgui/GL/gl3w.c.o
[ 20%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui_tables.cpp.o
[ 20%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui.cpp.o
[ 25%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui_draw.cpp.o
[ 29%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui_demo.cpp.o
[ 33%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui_impl_opengl3.cpp.o
[ 37%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui_impl_glfw.cpp.o
[ 41%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/imgui_widgets.cpp.o
[ 45%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/implot.cpp.o
[ 50%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/implot_demo.cpp.o
[ 54%] Building CXX object lib/CMakeFiles/GUI.dir/imgui/implot_items.cpp.o
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc: In member function ‘void leptrino::Comm_Rcv()’:
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:345:6: warning: unused variable ‘i’ [-Wunused-variable]
345 | int i,rt=0;
| ^
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc: In member function ‘int leptrino::Comm_SendData(UCHAR*, int)’:
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:299:7: warning: ignoring return value of ‘ssize_t write(int, const void*, size_t)’, declared with attribute warn_unused_result [-Wunused-result]
299 | write( fd, buff, l);
| ~~~~~^~~~~~~~~~~~~~
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc: In member function ‘bool leptrino::init()’:
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:160:33: warning: array subscript 4 is above array bounds of ‘SCHAR [4]’ {aka ‘signed char [4]’} [-Warray-bounds]
160 | stGetInfo->scFVer[F_VER_SIZE] = 0;
| ~~~~~~~~~~~~~~~~~~~~~~~~~~~~^
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:162:36: warning: array subscript 8 is above array bounds of ‘SCHAR [8]’ {aka ‘signed char [8]’} [-Warray-bounds]
162 | stGetInfo->scSerial[SERIAL_SIZE] = 0;
| ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:164:35: warning: array subscript 16 is above array bounds of ‘SCHAR [16]’ {aka ‘signed char [16]’} [-Warray-bounds]
164 | stGetInfo->scPName[P_NAME_SIZE] = 0;
| ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^
[ 58%] Linking CXX executable force_sensor_check
[ 58%] Built target force_sensor_check
Scanning dependencies of target check_force_sensor_com
[ 62%] Checking Leprino force sensor COM connection on /dev/ttyACM1
Get SensorInfo
Version:4009
SerialNo:2506007
Type:PFS080YA501U6

Start
[force_sensor_check] force sensor COM check passed
Application Close
[ 62%] Built target check_force_sensor_com
[ 66%] Linking CXX static library libGUI.a
[ 66%] Built target GUI
Scanning dependencies of target control
[ 70%] Building CXX object CMakeFiles/control.dir/src/gui.cc.o
[ 75%] Building CXX object CMakeFiles/control.dir/src/control_timer.cc.o
[ 79%] Building CXX object CMakeFiles/control.dir/src/thread_controller.cc.o
[ 83%] Building CXX object CMakeFiles/control.dir/src/leptrino.cc.o
[ 87%] Building CXX object CMakeFiles/control.dir/src/main.cc.o
[ 91%] Building CXX object CMakeFiles/control.dir/src/controller.cc.o
[ 95%] Building CXX object CMakeFiles/control.dir/src/system_controller.cc.o
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc: In member function ‘void leptrino::Comm_Rcv()’:
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:345:6: warning: unused variable ‘i’ [-Wunused-variable]
345 | int i,rt=0;
| ^
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc: In member function ‘int leptrino::Comm_SendData(UCHAR*, int)’:
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:299:7: warning: ignoring return value of ‘ssize_t write(int, const void*, size_t)’, declared with attribute warn_unused_result [-Wunused-result]
299 | write( fd, buff, l);
| ~~~~~^~~~~~~~~~~~~~
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc: In member function ‘bool leptrino::init()’:
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:160:33: warning: array subscript 4 is above array bounds of ‘SCHAR [4]’ {aka ‘signed char [4]’} [-Warray-bounds]
160 | stGetInfo->scFVer[F_VER_SIZE] = 0;
| ~~~~~~~~~~~~~~~~~~~~~~~~~~~~^
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:162:36: warning: array subscript 8 is above array bounds of ‘SCHAR [8]’ {aka ‘signed char [8]’} [-Warray-bounds]
162 | stGetInfo->scSerial[SERIAL_SIZE] = 0;
| ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^
/home/tomoya/sawaPC_shio_MotionController/src/leptrino.cc:164:35: warning: array subscript 16 is above array bounds of ‘SCHAR [16]’ {aka ‘signed char [16]’} [-Warray-bounds]
164 | stGetInfo->scPName[P_NAME_SIZE] = 0;
| ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^
/home/tomoya/sawaPC_shio_MotionController/src/gui.cc: In function ‘GLFWwindow* reset_and_open_window(std::string)’:
/home/tomoya/sawaPC_shio_MotionController/src/gui.cc:21:77: error: invalid conversion from ‘const GLFWmonitor*’ to ‘GLFWmonitor*’ [-fpermissive]
21 | const GLFWvidmode* mode = (primary_monitor != nullptr) ? glfwGetVideoMode(primary_monitor) : nullptr;
| ^~~~~~~~~~~~~~~
| |
| const GLFWmonitor*
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/gui.h:7,
from /home/tomoya/sawaPC_shio_MotionController/src/gui.cc:1:
/usr/include/GLFW/glfw3.h:2306:58: note: initializing argument 1 of ‘const GLFWvidmode* glfwGetVideoMode(GLFWmonitor*)’
2306 | GLFWAPI const GLFWvidmode* glfwGetVideoMode(GLFWmonitor\* monitor);
| ~~~~~~~~~~~~~^~~~~~~
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc: In lambda function:
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:173:60: error: binding reference of type ‘double&’ to ‘const double’ discards qualifiers
173 | mc::signal<double>::low_pass_filter(dt, cutoff, Fx(0), fx_in, Fx_buf(0));
| ^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:14,
from /home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:1:
/home/tomoya/sawaPC_shio_MotionController/./inc/signal_processing.h:10:73: note: initializing argument 4 of ‘static void mc::signal<T>::low_pass_filter(T, T, T&, T&, T&) [with T = double]’
10 | static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
| ~~~^~~~~
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:174:60: error: binding reference of type ‘double&’ to ‘const double’ discards qualifiers
174 | mc::signal<double>::low_pass_filter(dt, cutoff, Fy(0), fy_in, Fy_buf(0));
| ^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:14,
from /home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:1:
/home/tomoya/sawaPC_shio_MotionController/./inc/signal_processing.h:10:73: note: initializing argument 4 of ‘static void mc::signal<T>::low_pass_filter(T, T, T&, T&, T&) [with T = double]’
10 | static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
| ~~~^~~~~
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:175:60: error: binding reference of type ‘double&’ to ‘const double’ discards qualifiers
175 | mc::signal<double>::low_pass_filter(dt, cutoff, Fz(0), fz_in, Fz_buf(0));
| ^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:14,
from /home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:1:
/home/tomoya/sawaPC_shio_MotionController/./inc/signal_processing.h:10:73: note: initializing argument 4 of ‘static void mc::signal<T>::low_pass_filter(T, T, T&, T&, T&) [with T = double]’
10 | static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
| ~~~^~~~~
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:176:60: error: binding reference of type ‘double&’ to ‘const double’ discards qualifiers
176 | mc::signal<double>::low_pass_filter(dt, cutoff, Mx(0), mx_in, Mx_buf(0));
| ^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:14,
from /home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:1:
/home/tomoya/sawaPC_shio_MotionController/./inc/signal_processing.h:10:73: note: initializing argument 4 of ‘static void mc::signal<T>::low_pass_filter(T, T, T&, T&, T&) [with T = double]’
10 | static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
| ~~~^~~~~
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:177:60: error: binding reference of type ‘double&’ to ‘const double’ discards qualifiers
177 | mc::signal<double>::low_pass_filter(dt, cutoff, My(0), my_in, My_buf(0));
| ^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:14,
from /home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:1:
/home/tomoya/sawaPC_shio_MotionController/./inc/signal_processing.h:10:73: note: initializing argument 4 of ‘static void mc::signal<T>::low_pass_filter(T, T, T&, T&, T&) [with T = double]’
10 | static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
| ~~~^~~~~
/home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:178:60: error: binding reference of type ‘double&’ to ‘const double’ discards qualifiers
178 | mc::signal<double>::low_pass_filter(dt, cutoff, Mz(0), mz_in, Mz_buf(0));
| ^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:14,
from /home/tomoya/sawaPC_shio_MotionController/src/system_controller.cc:1:
/home/tomoya/sawaPC_shio_MotionController/./inc/signal_processing.h:10:73: note: initializing argument 4 of ‘static void mc::signal<T>::low_pass_filter(T, T, T&, T&, T&) [with T = double]’
10 | static void low_pass_filter(T dt, T cutoff_frequency, T &output, T &input, T &buf)
| ~~~^~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/src/main.cc:4:
/home/tomoya/sawaPC_shio_MotionController/./inc/leptrino.h:24:7: error: redefinition of ‘class leptrino’
24 | class leptrino
| ^~~~~~~~
In file included from /home/tomoya/sawaPC_shio_MotionController/./inc/system_controller.h:13,
from /home/tomoya/sawaPC_shio_MotionController/./inc/thread_controller.h:8,
from /home/tomoya/sawaPC_shio_MotionController/src/main.cc:3:
/home/tomoya/sawaPC_shio_MotionController/./inc/leptrino.h:24:7: note: previous definition of ‘class leptrino’
24 | class leptrino
| ^~~~~~~~
make[2]: **_ [CMakeFiles/control.dir/build.make:115: CMakeFiles/control.dir/src/gui.cc.o] Error 1
make[2]: _** Waiting for unfinished jobs....
make[2]: **_ [CMakeFiles/control.dir/build.make:102: CMakeFiles/control.dir/src/system_controller.cc.o] Error 1
make[2]: _** [CMakeFiles/control.dir/build.make:63: CMakeFiles/control.dir/src/main.cc.o] Error 1
make[1]: **_ [CMakeFiles/Makefile2:154: CMakeFiles/control.dir/all] Error 2
make: _** [Makefile:84: all] Error 2
