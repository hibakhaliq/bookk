---
sidebar_position: 3
---

# Chapter 3: Technical Integration

## Framework Integration Overview

This chapter focuses on Gazebo simulation with deep integration to other robotics frameworks:

- **Gazebo**: Core simulation environment and physics engine
- **ROS 2**: Integration for robot control and perception
- **Isaac**: Advanced simulation and perception capabilities
- **VSLAM**: Visual SLAM testing in simulated environments
- **VLA**: Multimodal simulation in virtual environments

## Gazebo Installation and Setup

### Installing Gazebo Garden

```bash
# Add Gazebo repository
sudo curl -sSL https://get.gazebo.dev | sh

# Install Gazebo Garden
sudo apt install gz-garden
```

### Installing ROS 2 Gazebo Packages

```bash
# Install gazebo_ros_pkgs
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

## Basic Gazebo World Creation

### World File Structure (SDF Format)

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="robotics_lab">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom models can be added here -->
    <model name="simple_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.02</iyy>
            <iyz>0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Robot Model Definition (URDF/SDF)

### Simple Differential Drive Robot (URDF)

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5707963267948966 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>
</robot>
```

## Gazebo-ROS 2 Integration

### Diff Drive Controller Plugin

```xml
<!-- Add to URDF/robot definition -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_wheel_tf>true</publish_wheel_tf>
  </plugin>
</gazebo>
```

## Sensor Integration

### Camera Sensor

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <pose>0.1 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <topic_name>camera/image_raw</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

### LIDAR Sensor

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topic_name>scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Launch File for Gazebo Integration

```python
# launch/gazebo_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import TextSubstitution
from launch.actions import LogInfo

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Start Gazebo server
    start_gazebo_cmd = Node(
        package='gz_ros2_control',
        executable='gz_ros2_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive_robot',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
```

## Isaac Integration

### NVIDIA Isaac Gazebo Plugins

```bash
# Install Isaac ROS for Gazebo integration
sudo apt install ros-humble-isaac-ros-gazebo
```

## VSLAM Integration in Simulation

### Running VSLAM in Gazebo

```bash
# Launch Gazebo with a robot that has camera sensors
# Then run VSLAM algorithms on the simulated camera data
ros2 launch rtabmap_ros demo_gazebo.launch.py
```

## VLA Integration

### Multimodal Simulation

Creating simulation environments for VLA (Visual-Language-Action) systems:

```xml
<!-- World with labeled objects for VLA training -->
<model name="labeled_object">
  <static>true</static>
  <link name="object_link">
    <visual name="visual">
      <geometry>
        <box>
          <size>0.2 0.2 0.2</size>
        </box>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.2 0.2 0.2</size>
        </box>
      </geometry>
    </collision>
  </link>
  <!-- Add semantic properties for VLA systems -->
  <sdf version="1.7">
    <extension>
      <semantic_label>box</semantic_label>
    </extension>
  </sdf>
</model>
```

## Hands-on Exercise: Simple Mobile Robot Simulation

Create a complete simulation environment with a differential drive robot:

1. Create URDF model for a simple robot with sensors
2. Create a Gazebo world file
3. Set up ROS 2 integration with diff drive controller
4. Add camera and LIDAR sensors
5. Create launch file to start the complete simulation

This exercise will help students understand the complete pipeline from robot modeling to simulation integration.