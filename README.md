# README for Epson IMU Driver for ROS2 Node

<!---toc start-->

* [README for Epson IMU Driver for ROS2 Node](#readme-for-epson-imu-driver-for-ros2-node)
  * [What is this repository for?](#what-is-this-repository-for)
  * [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
    * [For the UART Interface:](#for-the-uart-interface)
    * [For the SPI Interface:](#for-the-spi-interface)
  * [How do I use the driver?](#how-do-i-use-the-driver)
  * [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  * [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  * [How do I build, install, run this ROS package?](#how-do-i-build-install-run-this-ros-package)
    * [Example console output of *colcon build* for G366PDG0:](#example-console-output-of-colcon-build-for-g366pdg0)
    * [Example console output of launching ROS node for G366PDG0:](#example-console-output-of-launching-ros-node-for-g366pdg0)
  * [What does this ROS IMU Node Publish as messages?](#what-does-this-ros-imu-node-publish-as-messages)
    * [Without Quaternion Output](#without-quaternion-output)
      * [ROS Topic Message *data_raw*](#ros-topic-message-data_raw)
    * [With Quaternion Output](#with-quaternion-output)
      * [ROS Topic Message data](#ros-topic-message-data)
  * [Why am I seeing incorrect ROS timestamps or high latencies or slower than expected IMU data rates?](#why-am-i-seeing-incorrect-ros-timestamps-or-high-latencies-or-slower-than-expected-imu-data-rates)
    * [When using USB-UART bridges](#when-using-usb-uart-bridges)
      * [Modifying *latency_timer* by udev mechanism](#modifying-latency_timer-by-udev-mechanism)
      * [Modifying *latency_timer* by sysfs mechanism](#modifying-latency_timer-by-sysfs-mechanism)
      * [Modifying *low_latency* flag using *setserial* utility](#modifying-low_latency-flag-using-setserial-utility)
    * [When using the SPI interface](#when-using-the-spi-interface)
  * [Package Contents](#package-contents)
  * [License](#license)
  * [References](#references)

<!---toc end-->

## What is this repository for?

- This code provides communication between Epson IMU and ROS using the either the UART or SPI interface.
- For the UART connection, this code uses the standard Unix Terminal I/O library (termios) for communicating either direct or by USB-serial converter such as FTDI USB-UART bridge ICs.
- For the SPI connection, this code uses the [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) library for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux distro.
- The src/epson_imu_uart_ros2_node.cpp is the ROS C++ wrapper used to communicate with ROS using the UART interface.
- The src/epson_imu_spi_ros2_node.cpp is the ROS C++ wrapper used to communicate with ROS using the SPI interface.
- The other source files in src/ are based on the Linux C driver originally released by Epson:
  [Epson IMU Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
- Information about ROS2, and tutorials can be found: [ROS.org](https://index.ros.org/doc/ros2/)

## What kind of hardware or software will I likely need?

- [Epson IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/) (G320/G330/G354/G364/G365/G366/G370/V340)
- Epson IMU (G320/G330/G354/G364/G365/G366/G370/V340) [IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
- ROS2 Foxy or later (via download) [ROS.org](https://index.ros.org/doc/ros2/Installation/#installationguide)
- This software was developed and tested on the following:

```
  ROS2:        Foxy, Humble
  Description: Ubuntu 20.04 LTS, Ubuntu 22.04 LTS
```

### For the UART Interface:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect the IMU to ROS host (tty/serial) [See M-G32EV041](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Or alternatively, a direct connection from the IMU to the ROS platform that has 3.3V CMOS compatible UART interface [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/).

### For the SPI Interface:

- **NOTE:** This software is intended for use on an embedded Linux host system with a SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY).
  - For Raspberry Pi, the SPI interface should already be enabled using *raspi-config* or equivalent.
  - This code uses a separate GPIO to manually toggle CS# chipselect instead of the chipselect assigned to the HW SPI interface.
    - The assigned chipselect of the HW SPI interface should also work, but has not been thoroughly tested.
- Epson Breakout evaluation board or some equivalent to connect to ROS host (SPI & GPIOs) [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)

## How do I use the driver?

- This code assumes that the user is familiar with building ROS2 packages using the colcon build process.
- **NOTE:** This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS driver.
- Please refer to the ROS.org website for more detailed instructions on the ROS package build process. [ROS.org](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
- **NOTE:** At bare minimum, you must re-build the package with colcon after modifying the *CMakeLists.txt* such as changing the following

  - IMU model
  - serial interface type (UART or SPI)
  - host platform type (RPI or NONE=PC)

- If the IMU model is unchanged, then subsequent changes to IMU settings can be done by instead editing the IMU model specific launch.py file located in the launch/ folder
- The IMU model specific launch.py file should only be used in conjunction with the same colcon built executable of the same matching the IMU model.
- **NOTE:** Do not just switch launch.py files without modifying the CMakeLists.txt & rebuilding the executable to match the IMU model. Do not mix IMU model launch files without the matching IMU model colcon built binaries.

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the hcl_linux.c or hcl_rpi.c, there are wrapper functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
- On embedded Linux platforms, the user may need modify and redirect to platform specific delay routines if usleep() is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- When connecting the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET#, EXT, or DRDY is purely optional, and mainly intended for use with embedded Linux platforms (such as RapsberryPi).
- When connecting the IMU using the SPI interface, the use of GPIO pins for connecting to the IMU DRDY is mandatory (RESET# is recommended, EXT is optional).
- The user can choose to connect the IMU CS# pin directly with a designated host GPIO output or connect to the designated chipselect of the SPI interface (Default is SPI_CE0 pin on the RapsberryPi)
- When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization for better robustness.
- This code is structured for the user to easily redirect GPIO control to low-level hardware GPIO function calls for ease of implementation.
- There is no standard method to implement GPIO connections on embedded Linux platform, but the following files are examples of typical changes:

```
  src/hcl_linux.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires changes to hcl_linux.c (Use hcl_rpi.c as a template)

  - add #include to external library near the top of hcl_linux.c
  - add the initialization call inside the seInit() function in hcl_linux.c

For example on an Raspberry Pi, the following changes can be made to hcl_linux.c:

```
...

  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }

...
```

- Typically, the GPIO pins need to be assigned according to pin numbering specific to the embedded HW platform.
- This typically requires changes to hcl_gpio.h

**NOTE:** When using the SPI interface, if you will not control the CS# by host GPIO, then connect IMU CS# pin to the RPI SPI0_CE0, P1_24.

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
```

```
...

  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
...
```

- Typically, the external library will have GPIO pin control functions such as set_output(), set_input(), set(), reset(), read_pin_level(), etc...

- This requires changes to hcl_gpio.c (Use hcl_gpio_rpi.c as a template).

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.
  - For example on an Raspberry Pi, the following changes to hcl_gpio.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library

...

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }

...

  int gpioRelease(void)
  {
    return OK;
  }

...

  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  ...

  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  ...

  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  ...
```

## How do I build, install, run this ROS package?

The Epson IMU ROS2 driver is designed for building in the standard ROS2 colcon build environment.
Refer to the ROS2 Tutorials for more info: [ROS2 Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)

For more information on ROS & colcon setup refer to:
[Installing and Configuring ROS Environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/).

1. Place this package (including folders) into a new folder within your colcon workspace "src" folder.
   For example, we recommend using the folder name "ess_imu_driver2"

```
   <colcon_workspace>/src/ess_imu_driver2/
```

2. Modify the *CMakeLists.txt* to select the desired Epson IMU model, serial interface type, platform type that matches the attached ROS system.
   Refer to the comment lines inside the *CMakeLists.txt* for additional info.
   **NOTE:** You *MUST* re-build using *colcon build* when changing IMU models or after any changes in the *CMakeLists.txt*

3. From the colcon_workspace folder run *colcon build* to build all ROS2 packages located in the \<colcon_workspace>/src/ folder.
   **NOTE:** Re-run the above *colcon build* command to rebuild the driver after making any changes to the *CMakeLists.txt*, any of the .c or .cpp or .h source files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the approriate launch file, wherever possible, instead of modifying the .c or .cpp source files directly.

```
   <colcon_workspace>/colcon build --packages-select ess_imu_driver2 --symlink-install
```

4. Reload the current ROS environment variables that may have changed after the colcon build process.

```
   From the <colcon_workspace>: . install/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:

| Parameter            | Comment                                                                                                                                                     |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| frame_id             | specifies the value in the IMU message frame_id field                                                                                                       |
| imu_topic            | specifies the topic name for publishing IMU messages                                                                                                        |
| burst_polling_rate   | specifies the driver internal polling rate for the serial port input buffer (typically does not need changing)                                              |
| imu_dout_rate        | specifies the IMU output data rate                                                                                                                          |
| imu_filter_sel       | specifies the IMU filter setting                                                                                                                            |
| quaternion_output_en | enables or disables quaternion output (supported only be G330PDG0, G365PDC1, G365PDF1, G366PDG0)                                                            |
| atti_profile         | specifies the attitude motion profile (supported only be G330PDG0, G365PDC1, G365PDF1, G366PDG0)                                                            |
| output_32bit_en      | enables or disables outputing sensor data in 32 or 16 bit                                                                                                   |
| time_correction_en   | enables time correction function using IMU counter reset function & external 1PPS connection to IMU GPIO2/EXT pin. **Cannot be used when ext_trigger_en=1** |
| ext_trigger_en       | enables triggering of IMU samples using IMU external trigger function on IMU GPIO2/EXT pin. **Cannot be used when time_correction_en=1**                    |

**NOTE:** The ROS launch file passes IMU configuration settings to the IMU at runtime. Therefore, rebuilding with *colcon build* when changing the launch file is not required

**NOTE:** For previous ROS2 versions (Dashing or Eloquent) the launch.py needs to be modified to replace namespace parameter "executable" to "node_executable" because a change in ROS2 Foxy

6. To start the Epson IMU ROS2 driver use the appropriate launch.py file (located in launch/) from console. All parameters are described in the inline comments of the launch file. The launch file contains parameters for configuring IMU settings at runtime.

For example, for the Epson G366PDG0 IMU:

```
   <colcon_workspace>/ros2 launch ess_imu_driver2 g330_g365_g366_launch.py
```

| Launch File                  | Description                                                                                                                  |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| g320_g354_g364_launch.py     | For G320PDG0/G354PDH0/G364PDC0/G364PDCA outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)   |
| g330_g365_g366_launch.py     | For G330PDG0/G365PDx1/G366PDG0 outputs to ROS topic imu/data (**gyro, accel data, including quaternion orientation**)        |
| g330_g365_g366_raw_launch.py | For G330PDG0/G365PDx1/G366PDG0 outputs to ROS topic imu/data_raw (**gyro, accel data, but no quaternion orientation**)       |
| g370_launch.py               | For G370PDF1/G370PDS0/G370PDG0/G370PDT0 , outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**) |
| v340.launch.py               | For V340PDD0, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)                             |

### Example console output of *colcon build* for G366PDG0:

```
guest@VDC-RPI3-BPLUS:~/dev_ws$ colcon build --symlink-install --packages-select ess_imu_driver2
Starting >>> ess_imu_driver2
Finished <<< ess_imu_driver2 [2.01s]

Summary: 1 package finished [3.41s]
```

### Example console output of launching ROS node for G366PDG0:

```
user@VDC-RPI3B-PLUS:~/ros2_ws$ ros2 launch ess_imu_driver2 g330_g365_g366_launch.py
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2023-06-06-11-49-24-110969-VDC-RPI3B-PLUS-28405
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ess_imu_driver2_node-1]: process started with pid [28407]
[ess_imu_driver2_node-1] [INFO] [1686077365.780539432] [epson_node]: frame_id:              imu_link
[ess_imu_driver2_node-1] [INFO] [1686077365.781423229] [epson_node]: time_correction_en:    0
[ess_imu_driver2_node-1] [INFO] [1686077365.781555364] [epson_node]: ext_trigger_en:        0
[ess_imu_driver2_node-1] [INFO] [1686077365.781658853] [epson_node]: burst_polling_rate:    4000.0
[ess_imu_driver2_node-1] [INFO] [1686077365.781759009] [epson_node]: imu_dout_rate: 4
[ess_imu_driver2_node-1] [INFO] [1686077365.781889373] [epson_node]: imu_filter_sel:        5
[ess_imu_driver2_node-1] [INFO] [1686077365.782001143] [epson_node]: quaternion_output_en:  1
[ess_imu_driver2_node-1] [INFO] [1686077365.782081090] [epson_node]: imu_topic:             /epson_imu/data
[ess_imu_driver2_node-1] [INFO] [1686077365.782153017] [epson_node]: output_32bit_en:       1
[ess_imu_driver2_node-1] [INFO] [1686077365.782223069] [epson_node]: atti_profile:          0
[ess_imu_driver2_node-1]
[ess_imu_driver2_node-1] [INFO] [1686077365.805812625] [epson_node]: Checking sensor power on status...
[ess_imu_driver2_node-1] [INFO] [1686077366.633493532] [epson_node]: Initializing Sensor...
[ess_imu_driver2_node-1] [INFO] [1686077366.645052113] [epson_node]: Epson IMU initialized.
[ess_imu_driver2_node-1] [INFO] [1686077366.645299143] [epson_node]: Compiled for:  G366PDG0
[ess_imu_driver2_node-1] [INFO] [1686077366.645369299] [epson_node]: Reading device info...
[ess_imu_driver2_node-1] [INFO] [1686077366.647677151] [epson_node]: PRODUCT ID:    G330PDG0
[ess_imu_driver2_node-1] [INFO] [1686077366.650287657] [epson_node]: SERIAL ID:     T0000007
[ess_imu_driver2_node-1] [INFO] [1686077366.658343536] [epson_node]: OK: Build matches detected device
[ess_imu_driver2_node-1]
[ess_imu_driver2_node-1] [INFO] [1686077366.660287657] [epson_node]: Sensor started...
```

## What does this ROS IMU Node Publish as messages?

The Epson IMU ROS driver will publish messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models such as G320/G354/G364/G370/V340, the IMU messages **will only update the fields for angular rate (gyro) and linear acceleration (accel) data**.
- For IMU models G330/G365/G366, it depends on if the IMU attitude function with quaternion output enabled or not:
  - IMU messages will only update *angular_velocity* (gyro) and *linear_acceleration* (accel) fields when quaternion output is **disabled**, i.e. launch file *g330_g365_g366_raw.launch*
  - IMU messages will update update *angular_velocity* (gyro), *linear_acceleration* (accel), and *orientation* field using the internal extended Kalman Filter when the quaternion output is **enabled**, ie. launch file *g330_g365_g366.launch*
- Temperature sensor data from the IMU is also published on topic epson_tempc/ and will be remapped by the launch file to */imu/tempc*

### Without Quaternion Output

For non-quaternion output models, the ROS driver will publish to the following ROS topic:

```
/epson_imu/data_raw <-- orientation field will not contain valid data & should be ignored
```

#### ROS Topic Message *data_raw*

```
---
header:
  stamp:
    sec: 1685643703
    nanosec: 402378465
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.0002180843148380518
  y: 7.883128273533657e-05
  z: 6.580488116014749e-05
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.07670824229717255
  y: -0.056904129683971405
  z: -9.818550109863281
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

### With Quaternion Output

For quaternion output models, the ROS driver will publish to the following ROS topic:

```
/epson_imu/data <-- orientation, angular_velocity, linear_acceleration fields will be updating
```

#### ROS Topic Message data

```
---
header:
  stamp:
    sec: 1685644337
    nanosec: 660167310
  frame_id: imu_link
orientation:
  x: 0.9999865293502808
  y: -0.00023142248392105103
  z: 0.002697369083762169
  w: -0.004425106570124626
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.00032043419196270406
  y: -0.0005722396308556199
  z: 0.0002733084256760776
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.04799626022577286
  y: -0.07659434527158737
  z: -9.828039169311523
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

## Why am I seeing incorrect ROS timestamps or high latencies or slower than expected IMU data rates?

- By default, this software publishes the timestamp using ROS/Unix time for ROS IMU messages at the instant new IMU data is detected.
- Latencies from system overhead, system loading, and buffering will cause inaccuracies in this ROS timestamp.
- To improve timestamp accuracy, the user should consider adding GNSS receiver with 1PPS output to the ROS host system:
  - Consider setting up [chrony](https://chrony-project.org/examples.html) and [gpsd](https://gpsd.io/) with the GNSS receiver's 1PPS signal connected to the host system to improve host system clock accuracy
  - Consider also connecting the GNSS receiver's 1PPS signal to the Epson IMU GPIO2/EXT with *time_correction* enabled in the launch file to improve timstamp accuracy of the IMU sensor messages

### When using USB-UART bridges

- If your connection between the Epson IMU UART and the Linux host is by FTDI,
  the *latency_timer* setting in the FTDI driver may be large i.e. typically 16 (msec).
- There are 3 methods listed below to reduce the impact of this latency.

#### Modifying *latency_timer* by udev mechanism

- [udev](https://wiki.debian.org/udev) is a device manager for Linux that can dynamically create and remove devices in *userspace* and run commands when new devices appear or other events
- Create a udev rule to automatically set the *latency_timer* to 1 when an FTDI USB-UART device is plugged in
- For example, here is a text file named *99-ftdi_sio.rules* that can be put in the */etc/udev/rules.d* directory

```
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

**NOTE:** Requires root (sudo) access to create or copy file in */etc/udev/rules.d*
**NOTE:** This is the more robust method because it automatically sets when device is plugged in, but affects ALL FTDI USB-UART devices on the system

#### Modifying *latency_timer* by sysfs mechanism

- The example below reads the *latency_timer* setting for */dev/ttyUSB0* which returns 16msec.
- Then, it sets the *latency_timer* to 1msec, and confirms it by readback.

**NOTE: May require root (sudo su) access on your system to modify.**

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

#### Modifying *low_latency* flag using *setserial* utility

- The example below sets the *low_latency* flag for /dev/ttyUSB0.
- This will have the same effect as setting the *latency_timer* to 1msec.
- This can be confirmed by running the *setserial* command again.

```
user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user:~$ setserial /dev/ttyUSB0 low_latency

user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

### When using the SPI interface

- Issues with SPI interface will largely depend on your host system processing load and capabilities.
- If your ROS platform is running too many ROS node packages or simply too slow it may not react fast enough to detect the rising edge of the IMU DRDY signal and
  process the IMU sampling data.
- Try modifying the *dout_rate* and *filter_sel* to the slowest setting that can meet your ROS system requirements.
- Monitor the DRDY signal on the IMU with a oscilloscope to verify the systems processing reliability by monitoring the stability of the IMU DRDY signal and seeing that it matches the expected *dout_rate*.
- Try set the SPI clock rate to maximum of 1MHz (1000000), in the *\_node.cpp* for the spiInit() function call:

```
...

    while (rclcpp::ok() && !spiInit(SPI_MODE3, 1000000)) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the SPI layer in 1 second period...");
      one_sec.sleep();
    }

...
```

## Package Contents

The Epson IMU ROS2 driver-related sub-folders & root files are:

```
launch/        <== various example launch files for Epson IMU models
src/           <== source code for ROS2 node C++ wrapper, IMU Linux C driver, and additional README_src.md specifically for building and using the IMU Linux C driver as stand-alone (without ROS support)
CHANGELOG.rst  <== summarizes major changes
CMakeLists.txt <== cmake build script for colcon
LICENSE.txt    <== description of the applicable licenses
package.xml    <== colcon package description
README.md      <== general README
```

## License

Refer to *LICENSE.txt* in this package

## References

1. https://index.ros.org/doc/ros2/
2. https://github.com/technoroad/ADI_IMU_TR_Driver_ROS2
