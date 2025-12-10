---
sidebar_position: 5
---

# Chapter 2: Assessment

## Assessment Overview

This assessment evaluates your understanding of ROS 2 fundamentals, including package creation, communication patterns, services, actions, and system integration. The assessment includes multiple-choice questions, short answers, and practical exercises.

## Assessment Type
- **Format**: Mixed (Multiple Choice, Short Answer, Practical)
- **Duration**: 120 minutes
- **Passing Score**: 70%

## Multiple Choice Questions

### Question 1
What does DDS stand for in the context of ROS 2?
A) Distributed Data System
B) Data Distribution Service
C) Distributed Development System
D) Data Distribution Standard

### Question 2
Which build system is used for Python packages in ROS 2?
A) ament_cmake
B) catkin_make
C) ament_python
D) colcon

### Question 3
What is the primary difference between a ROS 2 service and a ROS 2 action?
A) Services are asynchronous, actions are synchronous
B) Actions provide feedback and can be canceled, services are request-response
C) Services use topics, actions use parameters
D) There is no difference

### Question 4
Which command is used to build a ROS 2 workspace?
A) ros2 build
B) colcon build
C) catkin_make
D) rosdep build

### Question 5
What is the purpose of a launch file in ROS 2?
A) To compile source code
B) To manage the lifecycle of nodes
C) To start multiple nodes with a single command
D) To debug node communication

## Short Answer Questions

### Question 6
Explain the differences between ROS 1 and ROS 2. What are the main advantages of ROS 2 over ROS 1?

### Question 7
Describe the three main communication patterns in ROS 2. For each pattern, explain when it should be used and provide an example scenario.

### Question 8
What is the purpose of the ROS 2 parameter system? How does it differ from using configuration files?

### Question 9
Explain the concept of ROS 2 lifecycle nodes. What are the different states a lifecycle node can be in?

### Question 10
Describe the structure of a typical ROS 2 package. What are the essential files and directories?

## Practical Exercise

### Exercise 1: Package Creation and Build
Create a ROS 2 package named "assessment_package" with the following specifications:
1. Use ament_python build type
2. Include a publisher node that publishes a custom message every 2 seconds
3. Include a subscriber node that receives and logs the message
4. Create a launch file that starts both nodes
5. Build the package and verify it compiles without errors

### Exercise 2: Service Implementation
Implement a ROS 2 service that calculates the distance between two points in 2D space:
1. Define a custom service interface with x, y coordinates for start and end points
2. Implement a service server that calculates and returns the Euclidean distance
3. Create a service client that calls the service with test coordinates
4. Test the service with multiple coordinate pairs

### Exercise 3: System Integration
Design a simple robot system with the following nodes:
- Sensor node (publishes sensor data)
- Controller node (subscribes to sensor data and publishes commands)
- Logger node (subscribes to all data and logs it)
- Use a launch file to start all nodes
- Verify that data flows correctly between nodes

## Answer Key

### Multiple Choice Answers:
1. B) Data Distribution Service
2. C) ament_python
3. B) Actions provide feedback and can be canceled, services are request-response
4. B) colcon build
5. C) To start multiple nodes with a single command

### Short Answer Rubric:
- Question 6: 15 points (5 points for each major difference)
- Question 7: 18 points (6 points for each communication pattern with examples)
- Question 8: 10 points (5 points for purpose, 5 points for differences)
- Question 9: 12 points (2 points for each lifecycle state)
- Question 10: 10 points (2 points for each essential component)

### Practical Exercise Rubric:
- Exercise 1: 20 points (5 points for each requirement met)
- Exercise 2: 15 points (5 points for interface, 5 for server, 5 for client)
- Exercise 3: 15 points (5 points for each node, 5 for launch file)

## Total Points: 115 (Extra credit opportunities included)

## Grading Scale:
- A: 100-115 points (87-100%)
- B: 85-99 points (74-86%)
- C: 70-84 points (61-73%)
- F: Below 70 points (Below 61%)