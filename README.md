# FIT Mar 2022 - ROS Robot Teleoperation

Generally commands to a robot can come from two different places, namely from an operator or a planner. These commands come in form of ideal linear and angular velocities that the operator or planner would like the robot to do. In 2D space these velocities are expressed by the $(v, \omega)$ pair. However, a robot is a physical device which has real, no-ideal motors for example. So, in either instance, operator or planner, the ideal commands must be translated to the non-ideal robot hardware, such that the robot can smoothly follow a trajectory, accelerate or decelerate.

## Teleoperation

Teleoperation allows you to control robot manually. There many are many different ways do this, such as: keyboard, joystick, QT teleop, interactive markers. Our exercises will cover keyboard functionality that must be implemented in order to drive the robot around smoothly. The hole implementation will be done in the `antrobot_key_teleop_node.py` script by following the TODO's in the file and the below exercise description. For building, running and testing the code consult the commands in the next section.

### Exercise 1 - Executing a target command

In this instance we are given a `target` value (i.e. linear/angular velocity) which the robot can't instantly do. So, the objective here is to implement a `__cmd_step__` method that will translate the `target` to an `output` that incrementally reaches the target in discrete steps. Each step has its own increment, or  more formally a specific `slope`. Identify the `__cmd_step__(...)` method within the skeleton located in the `antrobot_key_teleop_node.py` and complete its code.

### Exercise 2 - Thresholding a target command

Now that we can smoothly generate an output from a desired target we must ensure that target references issued by the operator or planner stay within the limits of what the physical system (i.e. the robot) can actually execute. Hence, you must implement the code for the `__threshold__(...)` method which we later use to transform a `target` value into an actual reference value that is within the two limits `low_limit` and `high_limit` (specified as the parameters of the function). Identify the `__threshold__(...)` method within the skeleton located in the `antrobot_key_teleop_node.py` and complete its code.

### Exercise 3 - Generating the linear velocity reference

With the `__threshold__(...)`  method implemented we can ensure that a target is in the physical range of what the robot can do. Here you'll have to use it to
ensure that `self.ref_linear_velocity` is within the limits of `[-self.max_linear_velocity, self.max_linear_velocity]`. Identify the `__set_ref_linear_velocity__(...)` method within the skeleton located in the `antrobot_key_teleop_node.py` and complete its code.

### Exercise 4 - Generating the angular velocity reference

As in exercise 3, here we'll have to ensure that the angular reference is within the physical range of what the robot can do. Identify the `__set_ref_angular_velocity__(...)` method within the skeleton located in the `antrobot_key_teleop_node.py` and complete its code.

### Exercise 5 - Generating References with the Keyboard

By now, we have all that we need to translate our keyboard inputs to actual references. For this task use `__set_ref_linear_velocity__(...)` / `__set_ref_angular_velocity__(...)`  together with `self.linear_step_size` / `self.angular_step_size` to increase or decrease the references for forward, backward, left, right motions. The keys that identify each motion, i.e. `w, s, a, d` are already parsed for you in the `run(...)` method. Identify the `run(...)` method within the skeleton located in the `antrobot_key_teleop_node.py` and complete its code for generating references for according to keys pressed.


## Building, Running and Testing Your Teleoperation
1. **Building the project**

To build the project you have to be located in the catkin workspace (i.e. `~/catkin_ws`)
and issue the command:

``catkin_make``

2. **Running your node**

To run your teleoperation node you must issue the following command:

``roslaunch antrobot_ros antrobot_teleop_key.launch``


3. **Running a simulation**

You can test your implementation with a simulated robot in Gazeboo environment:

``roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch``

## Notes

1. The ros documentation is located [here](http://wiki.ros.org/).
2. Hector slam wiki can be found [here](http://wiki.ros.org/hector_slam), while its code is located [here](https://github.com/tu-darmstadt-ros-pkg/hector_slam).
3. A useful resource for going beyond our simulation can be found [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).
