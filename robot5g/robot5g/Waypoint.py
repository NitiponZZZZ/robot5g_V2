#! /usr/bin/env python3

#x: -0.8038409352302551
#y: -0.3733225166797638
# yaw 0.0

import rclpy
from rclpy.node import Node
from .robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
import math


class navRobot(Node):
    def __init__(self):
        super().__init__('nav2_cmd_node')
        self.get_logger().info('Waypoint started')
        self.nav = BasicNavigator()

        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is activated')

        self.initial = PoseStamped()
        self.set_initial_pose()

        self.wp = []

    def set_initial_pose(self):
        self.initial.header.frame_id = "map"
        self.initial.header.stamp = self.get_clock().now().to_msg()
        self.initial.pose.position.x = -0.8038409352302551
        self.initial.pose.position.y = -0.3733225166797638

        q = self.quaternion_from_euler(0, 0, 0.0)

        self.initial.pose.orientation.w = q[0]
        self.initial.pose.orientation.x = q[1]
        self.initial.pose.orientation.y = q[2]
        self.initial.pose.orientation.z = q[3]

        self.nav.setInitialPose(self.initial)

    def set_point(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        q = self.quaternion_from_euler(0, 0, theta)

        goal_pose.pose.orientation.w = q[0]
        goal_pose.pose.orientation.x = q[1]
        goal_pose.pose.orientation.y = q[2]
        goal_pose.pose.orientation.z = q[3]

        return goal_pose

    def goto(self, x, y, theta):
        target = self.set_point(x, y, theta)
        self.nav.goToPose(target)

    def waypoint(self):
        pose = self.set_point(2.9363372325897217, 0.9414265751838684, 0.0)
        self.wp.append(pose)
        pose = self.set_point(2.5021910667419434, -1.763962984085083, 0.0)
        self.wp.append(pose)
        pose = self.set_point(-0.8038409352302551, -0.3733225166797638, 0.0)
        self.wp.append(pose)

        self.nav.followWaypoints(self.wp)

        while not self.nav.isNavComplete():
            print(self.nav.isNavComplete())
            feedback = self.nav.getFeedback()
            print(feedback)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        q = [0]*4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


def main():
    rclpy.init()
    nr = navRobot()
    # nr.goto(1.2,1.5,1.5707)
    nr.waypoint()
    nr.destroy_node()


if __name__ == "__main__":
    main()
