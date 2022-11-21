#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


class joyControl(Node):
    def __init__(self):
        super().__init__("joy_teleop")
        self.vel_L_pub = self.create_publisher(
            Float32, "wheel_command_left", 20)
        self.vel_R_pub = self.create_publisher(
            Float32, "wheel_command_right", 20)
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, 20)
        #self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback,qos_profile=qos.qos_profile_sensor_data)
        self.vel_timer = self.create_timer(0.05, self.vel_timer_callback)

        self.LeftHat = 0.0
        self.RightHatX = 0.0
        self.RightHatY = 0.0
        self.speedFW = 0.5
        self.speedT = 0.5
        self.L1_wheel_speed = 0.0
        self.R1_wheel_speed = 0.0
        self.L2_wheel_speed = 0.0
        self.R2_wheel_speed = 0.0

        self.get_logger().info('Joy Started')
        
    def joy_callback(self, joy_in):
        self.LeftHat = joy_in.axes[1]
        self.RightHatX = joy_in.axes[3]
        self.RightHatY = joy_in.axes[4]
        self.Enable = joy_in.buttons[4]
        self.AnaL2 = joy_in.axes[2]
        self.AnaR2 = joy_in.axes[5]

        if self.Enable:
            if self.LeftHat > 0.01 or self.LeftHat < -0.01:  # Forward & Backward
                self.L1_wheel_speed = self.LeftHat*self.speedFW
                self.R1_wheel_speed = self.LeftHat*self.speedFW
                self.L2_wheel_speed = self.LeftHat*self.speedFW
                self.R2_wheel_speed = self.LeftHat*self.speedFW

            if self.AnaL2 < -0.2 or self.AnaR2 < -0.2:
                if self.AnaL2 < -0.2:
                    #self.get_logger().info("Rotate Left")
                    self.L1_wheel_speed = -(-self.AnaL2*self.speedT)
                    self.R1_wheel_speed = (-self.AnaL2*self.speedT)
                    self.L2_wheel_speed = -(-self.AnaL2*self.speedT)
                    self.R2_wheel_speed = (-self.AnaL2*self.speedT)
                elif self.AnaR2 < -0.2:
                    #self.get_logger().info("Rotate Right")
                    self.L1_wheel_speed = (-self.AnaR2*self.speedT)
                    self.R1_wheel_speed = -(-self.AnaR2*self.speedT)
                    self.L2_wheel_speed = (-self.AnaR2*self.speedT)
                    self.R2_wheel_speed = -(-self.AnaR2*self.speedT)

            if self.RightHatX > 0.02 or self.RightHatX < -0.02 and self.LeftHat > 0.02 or self.LeftHat < -0.02:  # curve
                if self.RightHatX > 0.02 and self.LeftHat > 0.02:
                    #self.get_logger().info("Curve forward Left")
                    self.L1_wheel_speed = self.RightHatX*(self.speedT*0.5)
                    self.R1_wheel_speed = self.RightHatX*self.speedT
                    self.L2_wheel_speed = self.RightHatX*(self.speedT*0.5)
                    self.R2_wheel_speed = self.RightHatX*self.speedT

                elif self.RightHatX < -0.02 and self.LeftHat > 0.02:
                    #self.get_logger().info("Curve forward Right")
                    self.L1_wheel_speed = -1*self.RightHatX*self.speedT
                    self.R1_wheel_speed = -1*self.RightHatX*(self.speedT*0.5)
                    self.L2_wheel_speed = -1*self.RightHatX*self.speedT
                    self.R2_wheel_speed = -1*self.RightHatX*(self.speedT*0.5)

                elif self.RightHatX > 0.02 and self.LeftHat < -0.02:
                    #self.get_logger().info("Curve backward Left")
                    self.L1_wheel_speed = -(self.RightHatX*(self.speedT*0.5))
                    self.R1_wheel_speed = -(self.RightHatX*self.speedT)
                    self.L2_wheel_speed = -(self.RightHatX*(self.speedT*0.5))
                    self.R2_wheel_speed = -(self.RightHatX*self.speedT)

                elif self.RightHatX < -0.02 and self.LeftHat < -0.02:
                    #self.get_logger().info("Curve backward Right")
                    self.L1_wheel_speed = -(-self.RightHatX*self.speedT)
                    self.R1_wheel_speed = -(-self.RightHatX*(self.speedT*0.5))
                    self.L2_wheel_speed = -(-self.RightHatX*self.speedT)
                    self.R2_wheel_speed = -(-self.RightHatX*(self.speedT*0.5))
            elif (self.LeftHat < 0.01 and self.LeftHat > -0.01 and self.RightHatX < 0.02 and self.RightHatX > -0.02 and self.LeftHat < 0.02 and self.LeftHat > -0.02 and self.AnaL2 > -0.2 and self.AnaR2 > -0.2):
                # self.get_logger().info("Stop")
                self.L1_wheel_speed = 0.00
                self.R1_wheel_speed = 0.00
                self.L2_wheel_speed = 0.00
                self.R2_wheel_speed = 0.00

        else:
            self.L1_wheel_speed = 0.00
            self.R1_wheel_speed = 0.00
            self.L2_wheel_speed = 0.00
            self.R2_wheel_speed = 0.00

    def vel_timer_callback(self):
        vel_L_cmd = Float32()
        vel_R_cmd = Float32()
        vel_L_cmd.data = self.L2_wheel_speed  # FL
        vel_R_cmd.data = self.R2_wheel_speed  # FR

        self.vel_L_pub.publish(vel_L_cmd)
        self.vel_R_pub.publish(vel_R_cmd)


def main():
    rclpy.init()
    jc = joyControl()

    rclpy.spin(jc)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
