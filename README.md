# URC Intelligent Systems 2024

This repository contains the intelligent systems division's modules and sensor logic for SJSU Robotics' latest Mars Rover. It is based on [ROS Humble](https://docs.ros.org/en/humble/index.html).

# Getting Started

## Dependencies

To get started, make sure you have ROS Humble installed. You can check out the install instructions available [on the ROS website](https://docs.ros.org/en/humble/Installation.html). You will also need `colcon`, which can be installed according to [this ROS tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

## Placing the Repository

This repository should be cloned into your [ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)'s `src` directory. For example, if your ROS workspace was named `ros2ws`, your file tree should look like `ros2ws/src/urc_intelsys_2024`.

## Repository Structure

The repository is divided into several main directories.

- `launch` - this directory contains scripts that can be run using `ros2 launch urc_intelsys_2024 (script name)`
- `test` - this directory contains tests that can be run using `colcon test`
- `resource` - this directory should only contain the empty file `urc_intelsys_2024`. It is used purely for ROS 2's internals and shouldn't be altered.
- `urc_intelsys_2024` - this directory is where our main codebase resides.

## Contributing

- When contributing a submodule, remember that all submodules must have an `__init__.py` in order to be recognized as a submodule.
- When adding a new node, add its `main` function to the `entry_points` parameter in `setup.py` in the format `(name) = urc_intelsys_2024.(submodule1).(submodule2).(...).(file_name):main`.
