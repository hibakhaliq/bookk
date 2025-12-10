---
sidebar_position: 3
---

# Chapter 2: Technical Integration

## Framework Integration Overview

This chapter focuses on implementing ROS 2 fundamentals with practical examples. The technical integration emphasizes:

- **ROS 2**: Core concepts, packages, nodes, topics, services, and actions
- **Gazebo**: Integration with ROS 2 for simulation
- **Isaac**: ROS 2 bridges and interfaces
- **VSLAM**: ROS 2 packages for visual SLAM
- **VLA**: ROS 2 interfaces for multimodal systems

## ROS 2 Package Creation

### Creating a ROS 2 Package

```bash
# Create a new ROS 2 package
ros2 pkg create --build-type ament_python ros2_fundamentals_examples
```

### Package Structure

```
ros2_fundamentals_examples/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── ros2_fundamentals_examples
├── ros2_fundamentals_examples/
│   ├── __init__.py
│   └── ros2_examples.py
└── test/
    ├── __init__.py
    └── test_copyright.py
    └── test_flake8.py
    └── test_pep257.py
```

## Publisher-Subscriber Implementation

### Publisher Node

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

```python
# subscriber_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Implementation

### Service Definition

Create a service definition file `AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

### Service Server

```python
# service_member_function.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

### Basic Launch File

```python
# launch/example_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_fundamentals_examples',
            executable='publisher_node',
            name='publisher',
        ),
        Node(
            package='ros2_fundamentals_examples',
            executable='subscriber_node',
            name='subscriber',
        ),
    ])
```

## Gazebo Integration

### ROS 2 Gazebo Bridge

```bash
# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Launching Simulation

```xml
<!-- example_simulation.launch.xml -->
<launch>
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"/>
  <node pkg="ros2_fundamentals_examples" exec="robot_controller" name="robot_controller"/>
</launch>
```

## Isaac Integration

### ROS 2 to Isaac Bridge

Using the NVIDIA Isaac ROS packages for perception and navigation:

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-common
```

## VSLAM Integration

### ROS 2 VSLAM Packages

ROS 2 packages for visual SLAM:

```bash
# Install VSLAM packages
sudo apt install ros-humble-rtabmap-ros
```

## VLA Integration

### Multimodal Interface

Creating interfaces for VLA systems:

```python
# vla_interface.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

class VLAInterface(Node):
    def __init__(self):
        super().__init__('vla_interface')
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.command_publisher = self.create_publisher(
            String,
            'vla_commands',
            10)

    def image_callback(self, msg):
        # Process image and generate command
        command = String()
        command.data = "Processed image data for action"
        self.command_publisher.publish(command)
```

## Hands-on Exercise: Simple Robot Controller

Create a ROS 2 node that controls a simulated robot using publisher-subscriber pattern.

```bash
# Create the robot controller package
ros2 pkg create --build-type ament_python robot_controller_exercises
```

This exercise will help students understand ROS 2 communication patterns and robot control concepts.