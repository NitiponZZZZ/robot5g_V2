#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D, TransformStamped, Vector3
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import tf2_ros
import random

import math


class Robot(Node):
    def __init__(self):
        super().__init__('robot_core')
        self.wheel_radius = 0.1016  # 4 inches = 0.1016 m
        self.wheelbase = 0.475  # m

        self.max_rpm = 0.40  # rpm

        self.vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 10)

        self.leftwheel_pub = self.create_publisher(
            Float32, 'wheel_command_left', 10)
        self.rightwheel_pub = self.create_publisher(
            Float32, 'wheel_command_right', 10)
        self.seq_pub = self.create_publisher(
            Vector3, 'robotcheck_status', 10)
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, 20)

        self.v = 0.0
        self.w = 0.0

        self.timer = self.create_timer(0.04, self.timer_callback)

        self.cmd_l = Float32()
        self.cmd_r = Float32()

        self.left_tick_sub = self.create_subscription(
            Int32, 'left_tick', self.left_tick_cb, 10)
        self.right_tick_sub = self.create_subscription(
            Int32, 'right_tick', self.right_tick_cb, 10)

        self.left_tick = {"bf": None, "now": None}
        self.right_tick = {"bf": None, "now": None}
        self.left_tick_ts = None
        self.right_tick_ts = None

        self.last_time = self.get_clock().now()

        # Web
        self.web_control_sub = self.create_subscription(
            Twist, 'mode_control', self.web_control_callback, 10)
        self.cmd_l2 = Float32()
        self.cmd_r2 = Float32()
        self.Web_speed = 0.2
        self.Web_speedT = 0.2
        self.Web_Mode = 0
        self.Web_W = 0
        self.Web_A = 0
        self.Web_S = 0
        self.Web_D = 0

        # Joy
        self.cmd_l3 = Float32()
        self.cmd_r3 = Float32()
        self.LeftHat = 0.0
        self.RightHatX = 0.0
        self.RightHatY = 0.0
        self.speedFW = 0.5
        self.speedT = 0.5
        self.L1_wheel_speed = 0.0
        self.R1_wheel_speed = 0.0
        self.L2_wheel_speed = 0.0
        self.R2_wheel_speed = 0.0
        self.Enable = 0
        self.AnaL2 = 0.0
        self.AnaR2 = 0.0

        # Odometry
        self.tick_per_rev = 99000  # 100000
        self.robot_pose = Pose2D()  # x y theta
        self.dx = 0.0
        self.dy = 0.0
        self.dtheta = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # seq
        self.seq = Vector3()

        self.get_logger().info("robot core is running")

    def joy_callback(self, joy_in):
        self.LeftHat = joy_in.axes[1]
        self.RightHatX = joy_in.axes[3]
        self.RightHatY = joy_in.axes[4]
        self.Enable = joy_in.buttons[4]
        self.AnaL2 = joy_in.axes[2]
        self.AnaR2 = joy_in.axes[5]

    def web_control_callback(self, web_in):
        self.Web_Mode = int(web_in.linear.x)
        self.Web_W = int(web_in.linear.y)
        self.Web_A = int(web_in.linear.z)
        self.Web_S = int(web_in.angular.x)
        self.Web_D = int(web_in.angular.y)

    def left_tick_cb(self, tick_in):
        self.left_tick["now"] = tick_in.data
        self.left_tick_ts = self.get_clock().now()

    def right_tick_cb(self, tick_in):
        self.right_tick["now"] = tick_in.data
        self.right_tick_ts = self.get_clock().now()

    def get_tick(self):
        tick = {}
        tick["l"] = self.left_tick["now"]
        tick["r"] = self.right_tick["now"]
        tick["lts"] = self.left_tick_ts
        tick["rts"] = self.right_tick_ts

        return tick

    def vel_callback(self, vel):
        self.v = vel.linear.x
        self.w = vel.angular.z

        L = self.wheelbase
        R = self.wheel_radius

        v_r = ((2.0*self.v)+(self.w*L))/(2.0*R)
        v_l = ((2.0*self.v)-(self.w*L))/(2.0*R)

        self.wheel_speed_setpoint(v_r, v_l)

    def timer_callback(self):
        self.seq.y = float(random.randint(0, 100))
        self.seq_pub.publish(self.seq)
        if int(self.Web_Mode):
            if (self.Web_A and self.Web_W):
                self.cmd_l2.data = self.Web_speed*0.5
                self.cmd_r2.data = self.Web_speed

            elif (self.Web_D and self.Web_W):
                self.cmd_l2.data = self.Web_speed
                self.cmd_r2.data = self.Web_speed*0.5

            elif (self.Web_D and self.Web_S):
                self.cmd_l2.data = -self.Web_speed
                self.cmd_r2.data = -self.Web_speed*0.5

            elif (self.Web_A and self.Web_S):
                self.cmd_l2.data = -self.Web_speed*0.5
                self.cmd_r2.data = -self.Web_speed

            elif (self.Web_A):
                self.cmd_l2.data = -self.Web_speedT
                self.cmd_r2.data = self.Web_speedT

            elif (self.Web_D):
                self.cmd_l2.data = self.Web_speedT
                self.cmd_r2.data = -self.Web_speedT

            elif (self.Web_W):
                self.cmd_l2.data = self.Web_speed*0.8
                self.cmd_r2.data = self.Web_speed*0.8

            elif (self.Web_S):
                self.cmd_l2.data = -self.Web_speed*0.8
                self.cmd_r2.data = -self.Web_speed*0.8
            else:
                self.cmd_l2.data = 0.00
                self.cmd_r2.data = 0.00

            self.leftwheel_pub.publish(self.cmd_l2)
            self.rightwheel_pub.publish(self.cmd_r2)

        elif (not self.Web_Mode):
            if int(self.Enable):

                if self.LeftHat > 0.01 or self.LeftHat < -0.01:  # Forward & Backward
                    self.cmd_l3.data = self.LeftHat*self.speedFW
                    self.cmd_r3.data = self.LeftHat*self.speedFW

                if self.AnaL2 < -0.2 or self.AnaR2 < -0.2:
                    if self.AnaL2 < -0.2:
                        #self.get_logger().info("Rotate Left")
                        self.cmd_l3.data = -(-self.AnaL2*self.speedT)
                        self.cmd_r3.data = (-self.AnaL2*self.speedT)

                    elif self.AnaR2 < -0.2:
                        #self.get_logger().info("Rotate Right")
                        self.cmd_l3.data = (-self.AnaR2*self.speedT)
                        self.cmd_r3.data = -(-self.AnaR2*self.speedT)

                if self.RightHatX > 0.02 or self.RightHatX < -0.02 and self.LeftHat > 0.02 or self.LeftHat < -0.02:  # curve
                    if self.RightHatX > 0.02 and self.LeftHat > 0.02:
                        #self.get_logger().info("Curve forward Left")
                        self.cmd_l3.data = self.RightHatX*(self.speedT*0.5)
                        self.cmd_r3.data = self.RightHatX*self.speedT

                    elif self.RightHatX < -0.02 and self.LeftHat > 0.02:
                        #self.get_logger().info("Curve forward Right")
                        self.cmd_l3.data = -1*self.RightHatX*self.speedT
                        self.cmd_r3.data = -1*self.RightHatX*(self.speedT*0.5)

                    elif self.RightHatX > 0.02 and self.LeftHat < -0.02:
                        #self.get_logger().info("Curve backward Left")
                        self.cmd_l3.data = -(self.RightHatX*(self.speedT*0.5))
                        self.cmd_r3.data = -(self.RightHatX*self.speedT)

                    elif self.RightHatX < -0.02 and self.LeftHat < -0.02:
                        #self.get_logger().info("Curve backward Right")
                        self.cmd_l3.data = -(-self.RightHatX*self.speedT)
                        self.cmd_r3.data = -(-self.RightHatX*(self.speedT*0.5))

                elif (self.LeftHat < 0.01 and self.LeftHat > -0.01 and self.RightHatX < 0.02 and self.RightHatX > -0.02 and self.LeftHat < 0.02 and self.LeftHat > -0.02 and self.AnaL2 > -0.2 and self.AnaR2 > -0.2):
                    # self.get_logger().info("Stop")
                    self.cmd_l3.data = 0.00
                    self.cmd_r3.data = 0.00

                self.leftwheel_pub.publish(self.cmd_l3)
                self.rightwheel_pub.publish(self.cmd_r3)
            elif not int(self.Enable):
                # self.cmd_l3.data = 0.00
                # self.cmd_r3.data = 0.00
                self.leftwheel_pub.publish(self.cmd_l)
                self.rightwheel_pub.publish(self.cmd_r)

        if self.cmd_l.data != 0.00 and self.cmd_r.data != 0.00:
            self.get_logger().info('Auto left rpm: %s' % self.cmd_l.data)
            self.get_logger().info('Auto right rpm: %s' % self.cmd_r.data)
        if self.cmd_l2.data != 0.00 and self.cmd_r2.data != 0.00:
            self.get_logger().info('Manual left rpm: %s' % self.cmd_l2.data)
            self.get_logger().info('Manual right rpm: %s' % self.cmd_r2.data)
        self.odometry()

    def wheel_speed_setpoint(self, vr, vl):

        rpm_r = vr*9.549297
        rpm_l = vl*9.549297  # 1 rad/s = 9.549297 rpm

        rpm_r = max(min(rpm_r, self.max_rpm), -self.max_rpm)
        rpm_l = max(min(rpm_l, self.max_rpm), -self.max_rpm)

        if rpm_r == 0.0:
            self.cmd_r.data = 0.0
        if rpm_l == 0.0:
            self.cmd_l.data = 0.0

        self.cmd_r.data = rpm_r
        self.cmd_l.data = rpm_l

        # self.get_logger().info('out left rpm: %s' % rpm_l)
        # self.get_logger().info('out right rpm: %s' % rpm_r)

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

    def odometry(self):
        ts = self.get_clock().now()
        tick = self.get_tick()
        dt = ts - self.last_time
        dt = dt.nanoseconds*1e-9

        if tick['r'] == None or tick['l'] == None:
            return

        if self.right_tick["bf"] == None or self.left_tick["bf"] == None:
            self.right_tick['bf'] = tick['r']
            self.left_tick['bf'] = tick['l']

        prev_robot_pose = self.robot_pose
        R = self.wheel_radius
        L = self.wheelbase
        TPR = self.tick_per_rev

        DPT = (2 * math.pi*R)/TPR

        DL = DPT*(tick['l'] - self.left_tick['bf'])
        DR = DPT*(tick['r'] - self.right_tick['bf'])

        DC = (DL + DR)*0.5

        self.dx = DC*math.cos(prev_robot_pose.theta)
        self.dy = DC*math.sin(prev_robot_pose.theta)
        self.dtheta = (DR - DL)/L

        new_robot_pose = Pose2D()
        new_robot_pose.x = prev_robot_pose.x + self.dx
        new_robot_pose.y = prev_robot_pose.y + self.dy
        new_robot_pose.theta = prev_robot_pose.theta + self.dtheta

        self.robot_pose.x = new_robot_pose.x
        self.robot_pose.y = new_robot_pose.y
        self.robot_pose.theta = new_robot_pose.theta

        self.left_tick['bf'] = tick['l']
        self.right_tick['bf'] = tick['r']

        # publish odometry data
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.robot_pose.x
        odom.pose.pose.position.y = self.robot_pose.y
        odom.pose.pose.position.z = 0.0

        quat = self.quaternion_from_euler(0.0, 0.0, self.robot_pose.theta)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]

        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.robot_pose.x
        t.transform.translation.y = self.robot_pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]

        self.tf_broadcaster.sendTransform(t)

        self.last_time = ts


def main():
    rclpy.init()

    rb = Robot()
    rclpy.spin(rb)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
