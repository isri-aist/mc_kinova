# mc_kinova

mc_rtc robot module for Kinova Gen3 robot arms.

By default, only `Kinova` and `KinovaFloatingBase` are supported. To attach other tools (camera, gripper, force torque sensor, etc) to the end effector, install [mc_robot_tools](#dependencies) before building this module.

## Available robots variants

### Generic aliases

| Robot | Default tools used |
|-------|-------------------|
| `KinovaCamera` | RealSense D435 |
| `KinovaGripper` | Robotiq 2f-85 |
| `KinovaCameraGripper` | RealSense D435 + Robotiq 2f-85 |
| `KinovaBota` | BFT_SENS_ECAT_M8 sensor |
| `KinovaBotaDS4` | FT sensor + DS4 end effector |
| `KinovaBotaPlate` | FT sensor + Plate end effector |
| `KinovaBotaScrew` | FT sensor + Screw end effector |

#### Explicit variants

You can also specify exact tools — useful when you have multiple compatible modules. For example:
```
KinovaRealSenseD435
KinovaRobotiq2f85Gripper
KinovaRobotiqHandEGripper
KinovaRealSenseD435Robotiq2f85Gripper
KinovaBFT_SENS_ECAT_M8DS4
KinovaBFT_MEDS_ECAT_M8Plate
```

The naming convention is `Kinova<tool_module><end effector>`.

> **Note:** All robots have a floating base variant via `<robot-name>FloatingBase`. Bota variants also support `Callib` for force-torque sensor calibration.

## Dependencies

### Required
- [ROS2](https://docs.ros.org/)
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
- [kortex_description](https://github.com/Kinovarobotics/ros2_kortex)
### Optional
- [mc_robot_tools](https://github.com/isri-aist/mc_robot_tools)
  - [bota_driver](https://gitlab.com/botasys/drivers/bota_driver_ros2)
  - [robotiq_description](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/main/robotiq_description)
  - [robotiq_hande_description](https://github.com/macmacal/robotiq_hande_description)

To install `kortex_description`, `bota_driver`, `robotiq_description`, and `robotiq_hande_description`, you can use the following command:

```sh
mkdir -p ros2_ws/src && cd ros2_ws/src

git clone https://github.com/Kinovarobotics/ros2_kortex.git
# Optional — only needed if you want tool variants
git clone https://gitlab.com/botasys/drivers/bota_driver_ros2.git
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
git clone https://github.com/macmacal/robotiq_hande_description.git

cd ..
colcon build --symlink-install \
  --packages-select \
    kortex_description \
    bota_driver \
    robotiq_description \
    robotiq_hande_description
  --cmake-args -Wno-dev

source install/setup.bash
```

## Build and Install

```sh
git clone https://github.com/isri-aist/mc_kinova.git
cd mc_kinova
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ..
```
CMake will automatically detect mc_robot_tools if installed and enable the tool variants.

To test the module is installed correctly, run the following command from the root of the repository:

```sh
mc_rtc_ticker -f etc/mc_rtc.yaml
```
<p align="center">
  <img src="etc/kinova.png" alt="KinovaG3 mc_rtc_ticker test" height="500">
</p>

## Note: using `vcstool` with `.repos` files

This repository uses [vcstool](https://github.com/dirk-thomas/vcstool) ROS 2 dependency sources from `.repos` files. This keeps dependency setup reproducible and avoids manually cloning each repository. See the GitHub Actions workflows for examples.

- `default_dependencies.repos` contains the required ROS 2 dependencies.
- `extra_dependencies.repos` contains optional tool-related dependencies such as `bota_driver_ros2`, `ros2_robotiq_gripper`, and `robotiq_hande_description`, which are only required when building variants that depend on [mc_robot_tools](#dependencies).

To set up all ROS 2 dependencies:

```sh
mkdir -p /tmp/ros2_ws/src

vcs import --shallow --input default_dependencies.repos /tmp/ros2_ws/src
vcs import --shallow --input extra_dependencies.repos /tmp/ros2_ws/src

cd /tmp/ros2_ws
colcon build --symlink-install \
  --packages-select \
    kortex_description \
    bota_driver \
    robotiq_description \
    robotiq_hande_description
  --cmake-args -Wno-dev

source install/setup.bash
```

The `vcs import` commands clone each repository listed in the `.repos` files into the workspace source directory (`/tmp/ros2_ws/src`). You may replace this path with any desired ROS2 workspace location.
```
