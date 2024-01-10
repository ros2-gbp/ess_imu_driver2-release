cmake_minimum_required(VERSION 3.5)
project(ess_imu_driver2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Refer to the README_src.md inside the src folder for more details about IMU macro
# Uncomment the imu_model to build for
# NOTE: Change IMU models require rebuilding binaries with colcon

# Current
#set(IMU_MODEL "G330PDG0")
#set(IMU_MODEL "G365PDC1")
#set(IMU_MODEL "G365PDF1")
set(IMU_MODEL "G366PDG0")
#set(IMU_MODEL "G370PDF1")
#set(IMU_MODEL "G370PDS0")
#set(IMU_MODEL "G370PDG0")
#set(IMU_MODEL "G370PDT0")

# legacy
#set(IMU_MODEL "G320PDG0")
#set(IMU_MODEL "G354PDH0")
#set(IMU_MODEL "G364PDCA")
#set(IMU_MODEL "G364PDC0")
#set(IMU_MODEL "V340PDD0")

# Specify ACCL_RANGE_16G macro to enable 16G accelerometer range, otherwise defaults to 8G
# Only supported for G330PDG0/G366PDG0/G370PDG0/G370PDT0
# Uncomment below to enable 16G output range
#add_definitions(-DACCL_RANGE_16G)

# select NONE for standard PC, or RPI for RaspberryPi
set(PLATFORM "NONE")
#set(PLATFORM "RPI")

# select serial interface type
set(INTERFACE "UART")
#set(INTERFACE "SPI")

message(STATUS "-- Building for IMU Model: ${IMU_MODEL}")
message(STATUS "---- Building for platform: ${PLATFORM}")
message(STATUS "---- Building for interface: ${INTERFACE}")

# When SPI interface selected, mandatory platform is RPI
if (INTERFACE STREQUAL "SPI")
  set(PLATFORM "RPI")
  message(STATUS "---- SPI selected forcing platform: ${PLATFORM}")
endif()

# Define macros variables for compilation
add_definitions(-D${IMU_MODEL})
add_definitions(-DBUILD_FOR=\"${IMU_MODEL}\")
add_definitions(-D${PLATFORM})
add_definitions(-D${INTERFACE})

# Create file list for C libraries based on platform
if (PLATFORM STREQUAL "RPI")
  set(lib_sources
    src/hcl_rpi.c
    src/hcl_gpio_rpi.c
  )
elseif (PLATFORM STREQUAL "NONE")
  set(lib_sources
    src/hcl_linux.c
    src/hcl_gpio.c
  )
else()
  message(FATAL_ERROR "**** Invalid Platform")
endif()


# Create file list for C libraries based on interface
if (INTERFACE STREQUAL "SPI")
  set(lib_sources ${lib_sources}
    src/hcl_spi_rpi.c
    src/sensor_epsonCommon.c
    src/sensor_epsonSpi.c
  )
elseif (INTERFACE STREQUAL "UART")
  set(lib_sources ${lib_sources}
    src/hcl_uart.c
    src/sensor_epsonCommon.c
    src/sensor_epsonUart.c
  )
else()
  message(FATAL_ERROR "**** Invalid Interface")
endif()


# Add IMU model specific C source to filelist macro
if (IMU_MODEL STREQUAL "G320PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG320.c)
elseif (IMU_MODEL STREQUAL "G330PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG330_G366.c)
elseif (IMU_MODEL STREQUAL "G354PDH0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG354.c)
elseif (IMU_MODEL STREQUAL "G364PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (IMU_MODEL STREQUAL "G364PDCA")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (IMU_MODEL STREQUAL "G365PDC1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (IMU_MODEL STREQUAL "G365PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (IMU_MODEL STREQUAL "G366PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG330_G366.c)
elseif (IMU_MODEL STREQUAL "G370PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "G370PDS0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "G370PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "G370PDT0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "V340PDD0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonV340.c)
else()
  message(FATAL_ERROR "**** Invalid IMU Model")
endif()

# Declare library for Epson IMU functions
add_library(ess_imu_driver2_lib
  ${lib_sources}
)

# Link external libraries to Epson IMU Library
if (PLATFORM STREQUAL "RPI")
  # Determine location of wiringPi library on the host system
  # Needed if building on Raspberry Pi platform
  find_library(wiringPi_LIB NAMES wiringPi)

  target_link_libraries(ess_imu_driver2_lib
    ${wiringPi_LIB}
  )
endif()


# Declare a C++ executable
if (INTERFACE STREQUAL "SPI")
add_executable(ess_imu_driver2_node
  src/epson_imu_spi_ros2_node.cpp
)
elseif (INTERFACE STREQUAL "UART")
add_executable(ess_imu_driver2_node
  src/epson_imu_uart_ros2_node.cpp
)
else()
  message(FATAL_ERROR "**** Invalid Interface")
endif()


# Link C libraries to the ROS node
target_link_libraries(ess_imu_driver2_node
  ess_imu_driver2_lib
  crypt
  rt
  pthread
)

# Link C++ Library to the ROS node
ament_target_dependencies(ess_imu_driver2_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Mark executables and/or libraries for installation
install(
   TARGETS ess_imu_driver2_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

#ament_export_include_directories(include)
#ament_export_dependencies(std_msgs)
ament_package()
