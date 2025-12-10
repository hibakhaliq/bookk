---
sidebar_position: 4
---

# Chapter 2: Assignments

## Assignment 1: ROS 2 Package Creation and Management

### Description
Create a ROS 2 package that implements a simple robot status monitor. The package should include a publisher that broadcasts robot status information and a subscriber that listens to and processes this information.

### Requirements
- Create a new ROS 2 package using ament_cmake or ament_python
- Implement a publisher node that publishes robot status (battery level, position, etc.)
- Implement a subscriber node that receives and logs the status information
- Create a launch file that starts both nodes
- Document the package structure and functionality
- Include unit tests for your nodes

### Learning Outcomes
- Understanding of ROS 2 package structure
- Ability to create publisher and subscriber nodes
- Knowledge of launch file creation
- Understanding of ROS 2 build systems

### Estimated Time
4-5 hours

### Difficulty Level
Intermediate

## Assignment 2: Service-Based Robot Control

### Description
Implement a service-based control system for a simulated robot. The service should accept commands and return execution status.

### Requirements
- Define a custom service interface for robot commands
- Implement a service server that processes robot commands
- Create a service client that sends commands to the server
- Implement at least 3 different command types (move, stop, report status)
- Add error handling for invalid commands
- Test the service with various command scenarios

### Learning Outcomes
- Understanding of ROS 2 service architecture
- Ability to define custom service interfaces
- Knowledge of synchronous request-response communication
- Understanding of error handling in ROS 2

### Estimated Time
3-4 hours

### Difficulty Level
Intermediate

## Assignment 3: Action-Based Robot Navigation

### Description
Implement an action-based navigation system for a robot. Actions should be used for long-running navigation tasks with feedback.

### Requirements
- Define a custom action interface for navigation goals
- Implement an action server that handles navigation tasks
- Create an action client that sends navigation goals
- Implement feedback mechanism during navigation
- Include result reporting upon task completion
- Demonstrate cancellation of navigation goals

### Learning Outcomes
- Understanding of ROS 2 action architecture
- Knowledge of long-running task management
- Understanding of feedback and result mechanisms
- Ability to handle task cancellation

### Estimated Time
5-6 hours

### Difficulty Level
Intermediate/Advanced

## Assignment 4: Multi-Node System Integration

### Description
Design and implement a complete multi-node system that simulates a simple robot performing multiple tasks.

### Requirements
- At least 5 different nodes working together
- Use all three communication patterns (topics, services, actions)
- Implement a central coordinator node
- Include sensor simulation nodes
- Create an interface node for user interaction
- Use launch files to start the entire system
- Document the system architecture and node interactions

### Learning Outcomes
- Understanding of complex ROS 2 system design
- Ability to integrate multiple nodes and communication patterns
- Knowledge of system coordination and architecture
- Understanding of launch file complexity management

### Estimated Time
6-8 hours

### Difficulty Level
Advanced

## Assignment 5: ROS 2 Parameter Management

### Description
Implement a ROS 2 system with dynamic parameter management for robot configuration.

### Requirements
- Use ROS 2 parameter system for configuration
- Implement parameter callbacks for dynamic updates
- Create a parameter configuration file
- Demonstrate parameter updates during runtime
- Include parameter validation and error handling
- Document parameter usage and best practices

### Learning Outcomes
- Understanding of ROS 2 parameter system
- Knowledge of dynamic parameter updates
- Understanding of configuration management in ROS 2
- Ability to implement parameter validation

### Estimated Time
3-4 hours

### Difficulty Level
Intermediate