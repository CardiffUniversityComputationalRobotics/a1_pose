#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from champ_msgs.msg import ContactsStamped
from unitree_legged_msgs.msg import FeetForces
from geometry_msgs.msg import PolygonStamped
import math


class A1JointStates:
    def __init__(self):

        self.seq = 0

        # robot dimensions
        self.l1 = 0.0838

        self.l2 = 0.2

        self.l3 = 0.2

        # joint names
        # hip - thigh - calf

        self.joint_names_ = [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ]

        # ! publishers
        self.joint_state_pub_ = rospy.Publisher(
            "joint_states", JointState, queue_size=5
        )

        self.foot_contacts_pub_ = rospy.Publisher(
            "foot_contacts", ContactsStamped, queue_size=5
        )

        # ! subscribers
        self.foot_force_sub_ = rospy.Subscriber(
            "feet_forces", FeetForces, self.feet_force_cb
        )
        self.feet_pos_sub_ = rospy.Subscriber(
            "feet_polygon", PolygonStamped, self.feet_pos_cb
        )

        # ! messages
        self.joint_states_msg_ = JointState()
        self.joint_states_msg_.name = self.joint_names_

        self.feet_contacts_msg_ = ContactsStamped()

        self.rate = rospy.Rate(100)

    def feet_pos_cb(self, msg: PolygonStamped):

        leg_joints = []

        for pose in msg.polygon.points:
            leg_joint = self.get_leg_joint(pose.x, pose.y, pose.z)
            leg_joints = leg_joints + leg_joint

        self.joint_states_msg_.position = leg_joints

        leg_vels = [0] * 12
        self.joint_states_msg_.velocity = leg_vels

        leg_efforts = [0] * 12
        self.joint_states_msg_.effort = leg_efforts

    def feet_force_cb(self, msg: FeetForces):
        leg_contact = [False, False, False, False]

        if msg.fr_foot_force.force.z > 0.001:
            leg_contact[0] = True
        if msg.fl_foot_force.force.z > 0.001:
            leg_contact[1] = True
        if msg.rr_foot_force.force.z > 0.001:
            leg_contact[2] = True
        if msg.rl_foot_force.force.z > 0.001:
            leg_contact[3] = True

        self.feet_contacts_msg_.contacts = leg_contact

    def get_leg_joint(self, x, y, z, side="left"):

        # hip - thigh - calf

        D = (
            math.pow(x, 2)
            + math.pow(y, 2)
            - math.pow(self.l1, 2)
            + math.pow(z, 2)
            - math.pow(self.l2, 2)
            - math.pow(self.l3, 2)
        ) / (2 * self.l2 * self.l3)

        thetha1 = -math.atan2(-y, x) - math.atan2(
            math.sqrt(math.pow(x, 2) + math.pow(y, 2) - math.pow(self.l1, 2), -self.l1)
        )

        if side == "right":
            thetha3 = math.atan2(math.sqrt(1 - math.pow(D)), D)
        else:
            thetha3 = math.atan2(-math.sqrt(1 - math.pow(D, 2)), D)

        thetha2 = math.atan2(
            z, math.sqrt(math.pow(x, 2) + math.pow(y, 2) - math.pow(self.l1))
        ) - math.atan2(
            self.l3 * math.sin(thetha3), self.l2 + self.l3 * math.cos(thetha3)
        )

        return [thetha1, thetha2, thetha3]

    def run(self):
        while not rospy.is_shutdown():

            # foot contacts
            self.feet_contacts_msg_.header.seq = self.seq
            self.feet_contacts_msg_.header.stamp = rospy.Time.now()

            # joint states
            self.joint_states_msg_.header.seq = self.seq
            self.joint_states_msg_.header.stamp = rospy.Time.now()

            self.joint_state_pub_.publish(self.joint_states_msg_)
            self.foot_contacts_pub_.publish(self.feet_contacts_msg_)
            self.seq += 1
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("a1_joint_states_node")

    a1_joint_state_node = A1JointStates()
    a1_joint_state_node.run()
