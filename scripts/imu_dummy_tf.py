#!/usr/bin/env python
import rospy

import tf2_ros
import geometry_msgs.msg

from tf.transformations import quaternion_from_euler

if __name__ == "__main__":

    rospy.init_node("static_tf2_broadcaster")

    # xyz offset
    x_offset = rospy.get_param("~x_offset", 0)
    y_offset = rospy.get_param("~y_offset", 0)
    z_offset = rospy.get_param("~z_offset", 0)

    # rpy offset
    r_offset = rospy.get_param("~r_offset", 0)
    p_offset = rospy.get_param("~p_offset", 0)
    y_offset = rospy.get_param("~y_offset", 0)

    q_rot = quaternion_from_euler(r_offset, p_offset, y_offset)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "a1_gazebo/imu_link"
    static_transformStamped.child_frame_id = "a1_gazebo/imu_link_dummy"

    static_transformStamped.transform.translation.x = x_offset
    static_transformStamped.transform.translation.y = y_offset
    static_transformStamped.transform.translation.z = z_offset

    static_transformStamped.transform.rotation.x = q_rot[0]
    static_transformStamped.transform.rotation.y = q_rot[1]
    static_transformStamped.transform.rotation.z = q_rot[2]
    static_transformStamped.transform.rotation.w = q_rot[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
