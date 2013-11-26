monolithic_pr2_planner
======================

This is a full body planner for Willow Garage's PR2 robot. It computes a path to
take the robot from point A to B by controlling both arms, the base, and the
spine of the robot. 

The goal format for the planner is in terms of an object. This planner is
designed for the domain where the robot is trying to either pick up or put down
an object. Thus, designating a goal for the planner involves defining the X, Y,
Z, roll, pitch, and yaw of the object (or almost identically, the pose of the
end effector). The final position of the robot will place this object/gripper at
the given pose, while freely choosing the best spot for the base, arm, and
spine positions (while avoiding obstacles). 

This code is still under active development.

User's Guide
---------------------

Coming soon

Developer's Guide
---------------------

If you're looking to understand the planning theory, I've written tutorials
[here](http://sbpl.net/Tutorials).

If you'd like to read through the code for understanding, start in

    monolithic_pr2_planner/src/Environment.cpp

Assuming you're familiar with how SBPL works, you'll see functions for
environment initialization and also the main GetSuccs function. Documentation
and more tutorials are coming soon.


The entry point of this code is in 

    monolithic_pr2_planner_node/src/node.cpp

This is where everything is initialized and visible to ROS.

