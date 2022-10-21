# a1_pose

This package is in charge of publishing both the joints for the A1 real-world robot and additionally uses other packages to publish the odometry transform and data.

## Joints States

To publish the joint states, the node in `a1_joint_states.py` is used, which has the following publishers and subscribers.

### Publishers

- /joint_states ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))
- /foot_contacts ([champ_msgs/ContactsStamped](https://github.com/CardiffUniversityComputationalRobotics/champ/blob/master/champ_msgs/msg/ContactsStamped.msg))

  Shows which feet of the robot are supporting the robot against a surface.

### Subscribers

- /feet_forces ([unitree_legged_msgs/FeetForces](https://github.com/CardiffUniversityComputationalRobotics/unitree_ros_to_real/blob/main/unitree_legged_msgs/msg/FeetForces.msg))

  Expresses the force exerted in the foot of the robot in the Z axis.

- /feet_polygon ([geometry_msgs/PolygonStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PolygonStamped.html))

  Expresses the position of the end of the robot's feet.

## Odometry

Additionally, to estimate odometry, we use the node `state_estimation_node` from `champ_base` package which publishes the `odom` transform and the data to the topic `odom/raw`.
