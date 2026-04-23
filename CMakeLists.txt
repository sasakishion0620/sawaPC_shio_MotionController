cmake_minimum_required(VERSION 3.5)
project("TeleopHand")
set(TARGET_NAME "control")

include_directories(./inc/)
include_directories(./lib/)
include_directories(./lib/imgui)
add_subdirectory(lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")

option(USE_PCI_MODE "Enable PCI hardware mode" ON)
if(USE_PCI_MODE)
  add_definitions(-DPCI_MODE)
endif()

option(ENABLE_FORCE_SENSOR_COM_CHECK "Run Leprino COM connection check during build" ON)
set(FORCE_SENSOR_DEVICE "/dev/ttyACM1" CACHE STRING "Leprino force sensor serial device path")

# if not defined then timer will not wait.
add_definitions(-DREALTIME)

add_executable(${TARGET_NAME}
  ./src/main.cc
  ./src/control_timer.cc
  ./src/thread_controller.cc
  ./src/system_controller.cc
  ./src/gui.cc
  ./src/leptrino.cc
  ./src/force_target_generator.cc
  ./src/controller.cc
)

target_compile_definitions(${TARGET_NAME} PRIVATE LEPTRINO_DEVICE_PATH="${FORCE_SENSOR_DEVICE}")

add_executable(force_sensor_check
  ./src/force_sensor_check.cc
  ./src/leptrino.cc
)

target_compile_definitions(force_sensor_check PRIVATE LEPTRINO_DEVICE_PATH="${FORCE_SENSOR_DEVICE}")

if (UNIX)
  if(USE_PCI_MODE)
    target_link_libraries(${TARGET_NAME} GUI -lpthread -lGL -lrt -lglfw -lm -ldl stdc++fs -lpci)
    if(ENABLE_FORCE_SENSOR_COM_CHECK)
      add_custom_target(check_force_sensor_com ALL
        DEPENDS force_sensor_check
        COMMAND $<TARGET_FILE:force_sensor_check>
        COMMENT "Checking Leprino force sensor COM connection on ${FORCE_SENSOR_DEVICE}"
      )
    endif()
  else()
    target_link_libraries(${TARGET_NAME} GUI -lpthread -lGL -lrt -lglfw -lm -ldl stdc++fs)
  endif()
endif()
