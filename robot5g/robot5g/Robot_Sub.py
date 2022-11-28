#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from . import include_navi_data as navi_data
#from .include_navi_data import include_navi_data as navi_data


class ModeControl(Node):

    def __init__(self):
        super().__init__("MQTT_Reader")
        self.mode_pub = self.create_publisher(Twist, "mode_control", 10)
        self.stop_pub = self.create_publisher(Int16, "stop_control", 10)

        self.stop_timer = self.create_timer(0.05, self.stop_timer_callback)
        self.mode_timer = self.create_timer(0.05, self.mode_timer_callback)

    def mode_timer_callback(self):

        self.Mode_Control_ = navi_data.get_mode
        self.W = navi_data.key_w
        self.A = navi_data.key_a
        self.S = navi_data.key_s
        self.D = navi_data.key_d

        self.Mode = Twist()
        self.Mode.linear.x = float(self.Mode_Control_)
        self.Mode.linear.y = float(self.W)
        self.Mode.linear.z = float(self.A)
        self.Mode.angular.x = float(self.S)
        self.Mode.angular.y = float(self.D)

        self.mode_pub.publish(self.Mode)

    def stop_timer_callback(self):
        self.Stop = Int16()
        self.Stop.data = 0

        self.stop_pub.publish(self.Stop)


def main():
    rclpy.init()

    Mc = ModeControl()

    rclpy.spin(Mc)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
