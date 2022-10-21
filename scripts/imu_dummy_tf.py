#!/usr/bin/env python
import rospy

import tf2_ros
import geometry_msgs.msg

if __name__ == "__main__":

    rospy.init_node("static_tf2_broadcaster")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "a1_gazebo/imu_link"
    static_transformStamped.child_frame_id = "a1_gazebo/imu_link_dummy"

    static_transformStamped.transform.translation.x = float(0)
    static_transformStamped.transform.translation.y = float(0)
    static_transformStamped.transform.translation.z = float(-0.2)

    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
