# Basic Example Background

To make sure you understand the ins and outs of the structured testing framework, as well as ensuring it is compatible with your set up, a [basic_example](../test/basic_example/basic_test.sim) has been provided.

As seen in the graphic below, the example incorporates 1 - a custom built node called [Mono Increasing Publisher](../test/basic_example/test_publisher.py) located in the structured testing package itself, and 2 - the pre-existing _talker_ node that comes standard as part of the [ROS 2 examples](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/talker.py). They are unrelated but serve as an example of how the structured testing framework interacts both with pre-recorded data from a bag file as well as active nodes played from a launch file. 

To run the example, follow the steps in the _Running A Simulation_ section in the main [README](../README.md).

![Alt text](./BasicExampleDiagram.png?raw=true "Different Components of the Basic Example")