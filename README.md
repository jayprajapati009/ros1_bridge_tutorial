# Tutorial: Using `ros1_bridge` Package for ROS 1 (Noetic) and ROS 2 (Humble)

In this tutorial, you will learn how to use the [`ros1_bridge`](https://github.com/ros2/ros1_bridge) package to enable communication between ROS 1 (Noetic) and ROS 2 (Humble). This is useful when transitioning from ROS 1 to ROS 2, as it allows nodes from both ecosystems to exchange messages seamlessly.

## Dependencies

1. Ubuntu 20.04
2. ROS Noetic
3. ROS2 Humble (Built with source)
4. ros1_bridge (Built with source)

## Installation

1. Setup a Ubuntu 20.04 system. [Link](https://releases.ubuntu.com/focal/)
2. Install ROS Noetic. [Link](https://wiki.ros.org/noetic/Installation/Ubuntu)
3. Build ROS2 Humble Hawksbill from source. [link](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#using-the-ros-1-bridge)
(3rd step will take an hour, get some food)

## ros1_bridge Setup

For setting up the ros1_bridge package, create a `bridge_ws` workspace to build the package from source.

```sh
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src
```

```sh
git clone https://github.com/ros2/ros1_bridge.git
```

Before building the ROS1 bridge, ensure that everything else is built using standard Colcon arguments. It is not recommended to have the ROS1 environment sourced during this step, as doing so can add additional libraries to the path and potentially cause conflicts. Building your ROS 2 workspace without sourcing ROS1 environment variables will help maintain a clean and isolated environment for your ROS 2 packages.

```sh
cd ~/bridge_ws
colcon build --symlink-install --packages-skip ros1_bridge
```

Source the ROS1 environment now,

```sh
source /opt/ros/noetic/setup.bash
# the path may vary as per you installation
```

To ensure that the ROS1 bridge includes support for the necessary message/service packages, it's essential to add the relevant ROS1 and ROS2 workspaces that contain these packages to your environment's PATH. This can be achieved by explicitly specifying dependencies on the message/service packages in the `package.xml`` file of the bridge package. By doing this, Colcon will automatically include them in the path when building the bridge. Alternatively, you can manually source the relevant workspaces to make these packages accessible before building the bridge. This ensures that the bridge is aware of and capable of bridging the specified message/service packages between ROS1 and ROS2.

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

## Example 1

Let's try out an example where we initiate a ROS1 Talker (publisher) and a ROS2 Listener (subscriber) using `ros1_bridge`.

**Terminal-1:** First lets start a ROS1 `roscore`,

```sh
source /opt/ros/noetic/setup.bash
roscore
```

**Terminal-2:** The dynamic bridge, once started, functions by monitoring the availability of ROS1 and ROS 2 topics. When it identifies a matching topic in both ecosystems, it initiates the bridging process for messages on that specific topic. This allows communication and message exchange between ROS1 and ROS2 nodes, enabling interoperability between the two systems. Let's start the dynamic bridge.

```sh
# source the ROS1 environment first 
source /opt/ros/noetic/setup.bash
```

```sh
# now, source the ROS2 environment
source ~/ros2_humble/install/setup.bash
```

```sh
# source the bridge_ws
source ~/bridge_ws/install/setup.bash
```

```sh
# connect the ROS_MASTER_URI
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

The last command will start outputting the currently available topics in ROS1 and ROS2 in a regular interval.

**Terminal-3:** Here, we will initiate the ROS1 talker

```sh
source /opt/ros/noetic/setup.bash
rosrun rospy_tutorials talker
```

**Terminal-4:** Here, we will initiate the ROS2 listener

```sh
source ~/ros2_humble/install/setup.bash
ros2 run demo_nodes_cpp listener
```

If the ROS2 node (Terminal-4) starts printing the messages published by ROS1 node (Terminal-3). The ros1_bridge is working successfully.

## Example 2

Now, let's try out an example where we initiate a ROS2 Talker (publisher) and a ROS1 Listener (subscriber) using `ros1_bridge`.

**Terminal-1:** First lets start a ROS1 `roscore`,

```sh
source /opt/ros/noetic/setup.bash
roscore
```

**Terminal-2:** Let's start the dynamic bridge.

```sh
# source the ROS1 environment first 
source /opt/ros/noetic/setup.bash
```

```sh
# now, source the ROS2 environment
source ~/ros2_humble/install/setup.bash
```

```sh
# source the bridge_ws
source ~/bridge_ws/install/setup.bash
```

```sh
# connect the ROS_MASTER_URI
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

The last command will start outputting the currently available topics in ROS1 and ROS2 in a regular interval.

**Terminal-3:** Here, we will initiate the ROS2 talker

```sh
source ~/ros2_humble/install/setup.bash
ros2 run demo_nodes_py talker
```

**Terminal-4:** Here, we will initiate the ROS1 listener

```sh
source /opt/ros/noetic/setup.bash
rosrun roscpp_tutorials listener
```

## Example 3

In this example, we will demonstrate how the bridge can pass larger and more complex messages between ROS 1 and ROS 2. We'll set up a scenario where a ROS 2 node publishes images from a camera, and on the ROS 1 side, we'll use rqt_image_view to visualize these images in a graphical user interface (GUI). Additionally, a ROS 1 publisher can send a message to toggle an option in the ROS 2 node.

**Terminal-1:** First lets start a ROS1 `roscore`,

```sh
source /opt/ros/noetic/setup.bash
roscore
```

**Terminal-2:** Let's start the dynamic bridge.

```sh
source /opt/ros/noetic/setup.bash
source ~/ros2_humble/install/setup.bash
source ~/bridge_ws/install/setup.bash
# connect the ROS_MASTER_URI
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

The last command will start outputting the currently available topics in ROS1 and ROS2 in a regular interval.

**Terminal-3:** Here, we will initiate the ROS1 GUI

```sh
source /opt/ros/noetic/setup.bash
rqt_image_view /image
```

**Terminal-4:** Here, we will initiate the ROS2 image publisher from the `image_tools`

```sh
source ~/ros2_humble/install/setup.bash
ros2 run image_tools cam2image
```

## References

[1] <https://github.com/ros2/ros1_bridge#example-1a-ros-1-talker-and-ros-2-listener>
[2] <https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS1-ROS2-bridge.html>
[3] <https://www.theconstructsim.com/how-to-communicate-between-ros1-ros2-with-ros1_bridge/>

>This tutorial should help you get started with using the ros1_bridge package to enable communication between ROS 1 (Noetic) and ROS 2 (Humble) nodes. You can extend this knowledge to adapt more complex ROS 1 nodes to ROS 2 or vice versa as needed for your projects.
