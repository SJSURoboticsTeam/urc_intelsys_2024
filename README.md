# URC Intelligent Systems 2024

This repository contains the intelligent systems division's modules and sensor logic for SJSU Robotics' latest Mars Rover. It is based on [ROS Humble](https://docs.ros.org/en/humble/index.html), although it can be (probably) used with [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html) as well.

# Architecture

![Diagram](/architectures/diagram.png)

- Sensors publish their raw data to their respective topics
- Map nodes process that data and publish a 2d array with obstacles and free spaces to /map
- Control nodes subscribe to /map and any raw data that they need to in order to publish goals to /goal
- Pathfinder nodes subscribe to /map and /goal and push command requests to the CAN bus

# Getting Started

## Placing the Repository

This repository is already a [ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) with a complete `src` directory. This means several things:

- You do not need to clone this repository into an existing ROS workspace.
- All ROS commands should be executed from the root of this repository.

## Dependencies

To get started, make sure you have ROS Humble installed. You can check out the install instructions available [on the ROS website](https://docs.ros.org/en/humble/Installation.html). You will also need `colcon`, which can be installed according to [this ROS tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html). Finally, make sure that you install all the repository-specific dependencies by running

```
rosdep install --from-paths src -y --ignore-src
```

More details about rosdep can be found on the [ROS rosdep tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html).

## Repository Structure

The repository is already a ROS workspace. As such, it contains a `src` directory with individual packages. Each individual package has the following subdirectories:

- `launch` (optional) - this directory contains scripts that can be run using `ros2 launch (package name) (script name)`. This is only present in the `urc_intelsys_2024` package.
- `msg` (optional) - this directory contains message definitions that allow us to pass structured data over topics. This is only present in the `urc_intelsys_2024_msg` package.
- `test` - this directory contains tests that can be run using `colcon test`
- `resource` - this directory should only contain the empty file `(package name)`. It is used purely for ROS 2's internals and shouldn't be altered.
- `(package name)` - this directory is where the code for the package goes.

## Workflow

### Building

From the root of this repository, you can use colcon to build the ROS packages. For Python packages, this simply bundles all the files together and puts them into the install directory. For C++ packages, this actually compiles and links any executables in the package.

### Building all Packages

To build all packages, run

```sh
colcon build
```

### Building Selected Packages

To build only certain packages, you can use the `--packages-select` argument followed by the packages you'd like to build.

```sh
colcon build --packages-select compass gps
```

### Installing

Once built, there should be an `install` directory present in the root of this repository. To install the built packages, simply run

```sh
source install/setup.sh
```

If you have Zshell, you'll run

```sh
source install/setup.zsh
```

### Running

After installing, scripts and launch files can be run with `ros2 run` and `ros2 launch`, respectively.

#### Scripts

Scripts are the entry points into our applications. They are often single nodes, but sometimes are more than one node. We define the scripts for a package in `src/(package name)/setup.py`, and then we can run those scripts with the following command:

```sh
ros2 run (package name) (entry_point_name)
```

For example, to run the GPS publisher, you can run

```sh
ros2 run gps gps
```

To pass arguments to a node, you can specify the `--ros-args -p` flag followed by all the arguments that you want to pass, in the format `argument_name:=argument_value`.

#### Launch Files

Launch files (defined in `src/urc_intelsys_2024/launch/*.py`) act as a way to launch multiple nodes at the same time. To run a launchfile, run the following command:

```sh
ros2 launch urc_intelsys_2024 (launch_file_name).py
```

For example, to run all sensors, you can run

```sh
ros2 launch urc_intelsys_2024 sensor_launch.py
```

To pass arguments (for example, to specify that all sensors should be fake), you can specify `(argument_name):=(value)`. For example:

```sh
ros2 launch urc_intelsys_2024 sensor_launch.py compass_type:=fake gps_type:=fake
```

Alternatively, you can also edit the values in the config file, found in `sr/urc_intelsys_2024/config/config.yaml` and re-build the `urc_intelsys_2024` package.

To learn more about launchfiles, see [the ROS documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

## Contributing

To add a new Python package, you can run

```sh
ros2 pkg create --build-type ament_python (package_name)
```

Then, to add an executable node, edit the file `setup.py` within the package. Edit the `entry_points` variable as follows:

```py
setup(
    ...
    # edit entry_points:
    entry_points={
        "console_scripts": [
            "node_name = package.file:function",
            ...  # add any other nodes you have
        ]
    }
)
```

To learn more about this, check out the ROS2 documentation for [creating packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

Before pushing your code, please run `python3 -m black ./src` to format your code. Otherwise, your code will fail formatting checks.
