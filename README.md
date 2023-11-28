![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

# Amanita Robot Firmware based on micro-ROS component for ESP-IDF

This firmware eas tested with ESP-IDF v5.1.2 running on an ESP32 WROOM board. It might work with other ESP32 boards too.

The micro-ROS component for ESP-IDF is available on https://github.com/micro-ROS/micro_ros_espidf_component.

## Dependencies

This component needs `colcon` and other Python 3 packages inside the IDF virtual environment in order to build micro-ROS packages:

```bash
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```

## Clone repo and submodules

You can clone this repo and open the generated folder in VSCode with DevContainer plugin installed. Build the container and reopen VSCode in remote mode. All the ESP-IDF commands work on the Docker container terminal.

```bash
git clone --recurse-submodules git@github.com:branilson/amanita-uros-espidf.git
```

## How-to

In order to test the Amanita Robot Firmware, connect the board in a USB port and run the commands below:

### Set target board [esp32|esp32s2|esp32s3|esp32c3]
```bash
idf.py set-target esp32
```
### Set your micro-ROS configuration and WiFi credentials under micro-ROS Settings
```bash
idf.py menuconfig
```
### Build the firmware
```bash
idf.py build
```
### Flash the microcontroller
```bash
idf.py flash
```
### Serial monitoring
```bash
idf.py monitor
```
### Clean and rebuild all the micro-ROS library
```bash
idf.py clean-microros
```
### Full Clean and configuration reset
```bash
idf.py fullclean
```

It's possible to use a micro-ROS Agent just with this docker command:

## Testing with ROS2

### UDPv4 micro-ROS Agent
```bash
docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888 -v6
```

### Send a Twist message using ROS2
```bash
docker run -it --rm osrf/ros:humble-desktop ros2 topic pub --once /amanita/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.8}}"
```

## Build in the CLI with docker container

It's possible to build this example application using preconfigured docker container. Execute this line to build an example app using docker container:

```bash
docker build -t espidf-microros .devcontainer/
```
```bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/workspaces/amanita-uros-espidf -v /dev:/dev --device-cgroup-rule='c *:* rmw' --workdir /workspaces/amanita-uros-espidf espidf-microros /bin/bash  -c "idf.py menuconfig build flash monitor"
```

Dockerfile for this container is provided in the ./.devcontainer directory and it is based on espressif/idf:v5.1.2, available in dockerhub.

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
