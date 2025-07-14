# Navigation 2

## Description
The "nav2" package is an essential package within the ROS 2 ecosystem, primarily focused on providing navigation and path planning capabilities for mobile robots and autonomous systems. Navigation 2 builds upon the lessons learned from the original Navigation Stack in ROS and aims to offer a more flexible and scalable navigation solution for a wide range of robotic platforms. You can find the official nav2 documentation [here](
https://navigation.ros.org/).


This repository contains packages for the configuration and launch of the Navigation 2 stack. Besides that, that are scripts to run the Navigation 2 stack in a docker container.

## Building
To build the dockerized implementation of the nav2 framework **locally**, execute:
```
git clone https://git-ce.rwth-aachen.de/wzl-mq-ms/docker-ros/ros2/nav2-ros.git
cd nav2-ros
bash docker_build_kairos.sh
```
The building process on the robots or on the edge system is handeled by the [.gitlab-ci.yml](./.gitlab-ci.yml). The CI/CD process is automatically initiated by pushing code changes to this repository. You can view the pipleline status by clicking `CI/CD` --> `Pipelines` on the left.

After building locally you should find a docker image called `nav2-ros:kairos-main` on your host. If you used the `.gitlab-ci.yml` for building the image is called `hades:5000/nav2-ros:kairos-main` or `registry.git-ce.rwth-aachen.de/wzl-mq-ms/docker-ros/ros2/nav2-ros:kairos-main`.
To check if the building process has been successful, execute:
```
docker images | grep nav2-ros
```
Before continuing with the "Usage" section, make sure that you have `tmux` installed:

```
sudo apt-get install tmux
```

## Usage
### Before executing any run script:
Make sure that:
- All ROS2 software modules running **on the same host** have the same `ROS_DOMAIN_ID` specified.
- All ROS2 software modules running **on all hosts** have the same `RMW_IMPLEMENTATION` specified.
- All ip addresses and usernames in the run script are specified and reachable. Check that by executing `ssh [USERNAME]@[IP_ADDRESS]`. If you are asked to enter a password, follow this [documentation](https://www.ibm.com/support/pages/configuring-ssh-login-without-password ) to allow ssh without password
- The image name matches your `docker images | grep nav2-ros` output.
- The robot (simulated or real) is running and publishing LiDAR/lasercan and odometry topics.

### Running the navigation 2 stack in a docker container
To run the navigation 2 stack in a docker container using the provided scripts, you need to have **docker** and **tmux** installed. After that, you can run one of the following scripts from inside the `docker_run` folder:

- [run_navigation.sh](./docker_run/run_navigation.sh): Starts all the necessary containers, in a specified host, to run the navigation 2 stack with a real robot.
- [run_navigation_vis.sh](./docker_run/run_navigation_vis.sh): Starts all the necessary containers to run the navigation 2 stack with a real robot. Rviz is started in a specified visualization host and all the other containers are started in another specified host.
- [run_navigation_sim.sh](./docker_run/run_navigation_sim.sh): Starts all the necessary containers, in the computer that is running the scripts, to run the navigation 2 stack with a simulated robot.
- [run_navigation_local.sh](./docker_run/run_navigation_sim.sh): Starts all the necessary containers, in the computer that is running the scripts, to run the local built navigation 2 stack with a real robot. This script is useful for development.

All the configuration for the scripts can be found in the begging of each script.

### Running the navigation 2 stack locally
It is possible to run the navigation 2 stack locally instead of running it using docker, besides ROS 2, you need to install the packages dependencies in your computer. You can do that by running the following command from inside the `nav_ws` folder:

```bash
rosdep install --from-paths src --ignore-src
```

After that, you can build the packages by running the following command from inside the `nav_ws` folder:

```bash
colcon build --symlink-install
```

Then you can source the workspace and to run the navigation 2 stack, you can run the following command:

```bash
ros2 launch kairos_navigation bringup_launch.py
```

After that, to set the initial pose of the robot, you can run the following command:

```bash
ros2 run kairos_navigation set_initial_pose_launch.py
```

Finally, to open the rviz2 interface, you can run the following command:

```bash
ros2 launch kairos_navigation rviz_launch.py
```

### Running the gazebo simulation

To run the gazebo simulation, you need to have docker and docker-compose installed. After installing it, the simulation can be run using the devcontainer or using docker-compose.

The simulation is based on a test world, which you can see its definition in the [.devcontainer/test.world](./.devcontainer/test.world) file.


#### Using the devcontainer

This project contains a [devcontainer.json](./.devcontainer/devcontainer.json) file, which is used by [VSCode Dev Containers](https://code.visualstudio.com/docs/remote/containers) to create a development environment inside a container. To use it, install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension and open the project in VSCode. Use the command `Dev-Containers: Reopen in Container` in the VSCode Command Palette to open the project in the container.

When running the devcontainer, a simulation container will be created and the simulation will be started.

#### Using docker-compose

Alternatively, you can run the simulation using docker-compose. To do so you can run the following command from inside the `.devcontainer` folder:

```bash
docker compose up --build
```

## Known Problems

### Transform Data too old Error
- Error Message:
```
[controller_server-4] [ERROR] [1696313944.345164184] [tf_help]: Transform data too old when converting from kairosAB_odom to map
[controller_server-4] [ERROR] [1696313944.345190556] [tf_help]: Data time: 1696313944s 281747712ns, Transform time: 1696313943s 992789500ns
[controller_server-4] [WARN] [1696313944.345223023] [kairosAB.controller_server]: Unable to transform robot pose into global plan's frame
[controller_server-4] [ERROR] [1696313944.345271358] [kairosAB.controller_server]: Controller patience exceeded
[controller_server-4] [WARN] [1696313944.345318112] [kairosAB.controller_server]: [follow_path] [ActionServer] Aborting handle.
```
- Solution:
No solution found yet.