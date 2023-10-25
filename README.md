# Tutorial: Using `ros1_bridge` Package for ROS 1 (Noetic) and ROS 2 (Humble)

In this tutorial, you will learn how to use the [`ros1_bridge`](https://github.com/ros2/ros1_bridge) package to enable communication between ROS 1 (Noetic) and ROS 2 (Humble). This is useful when transitioning from ROS 1 to ROS 2, as it allows nodes from both ecosystems to exchange messages seamlessly.

## Dependencies

1. Ubuntu 20.04
2. ROS Noetic
3. ROS2 Humble (Built with source)
4. ros1_bridge (Built with source)

## Steps

### Installation

1. Setup a Ubuntu 20.04 system. [Link](https://releases.ubuntu.com/focal/)
2. Install ROS Noetic. [Link](https://wiki.ros.org/noetic/Installation/Ubuntu)
3. Build ROS2 Humble Hawksbill from source. [link](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#using-the-ros-1-bridge)
(3rd step will take an hour, get some food)

### ros1_bridge Setup

For setting up the ros1_bridge package, create a `bridge_ws` workspace to build the package from source.

```sh
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src
```

```sh
git clone https://github.com/ros2/ros1_bridge.git
```

Before building the ROS 1 bridge, ensure that everything else is built using standard Colcon arguments. It is not recommended to have the ROS 1 environment sourced during this step, as doing so can add additional libraries to the path and potentially cause conflicts. Building your ROS 2 workspace without sourcing ROS 1 environment variables will help maintain a clean and isolated environment for your ROS 2 packages.

```sh
cd ~/bridge_ws
colcon build --symlink-install --packages-skip ros1_bridge
```

Source the ROS1 environment now,

```sh
source /opt/ros/noetic/setup.bash
# the path may vary as per you installation
```

To ensure that the ROS 1 bridge includes support for the necessary message/service packages, it's essential to add the relevant ROS 1 and ROS 2 workspaces that contain these packages to your environment's PATH. This can be achieved by explicitly specifying dependencies on the message/service packages in the `package.xml`` file of the bridge package. By doing this, Colcon will automatically include them in the path when building the bridge. Alternatively, you can manually source the relevant workspaces to make these packages accessible before building the bridge. This ensures that the bridge is aware of and capable of bridging the specified message/service packages between ROS 1 and ROS 2.

Currently we source it manually,

```sh
source ~/ros2_humble/install/setup.bash
# the path may vary as per you installation
```

**Note:** Whenever you source ROS1 after sourcing ROS2 or vice-versa, you might observe the follwoing output in the terminal.
(relax and ignore)

```text
ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions.
```

Finally, built the ros1_bridge, it make take around 12 minutes to build.
(time to get a coffee)

```sh
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```
