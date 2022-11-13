#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos
import serial
from std_msgs.msg import Int32, Float32

ttl = serial.Serial(port='/dev/ttl_arduino', baudrate=115200)


class bringup(Node):
    def __init__(self):
        super().__init__("bringup")
        self.vel_L_sub = self.create_subscription(Float32, "wheel_command_left", self.L_callback,
                                                  qos_profile=qos.qos_profile_sensor_data)
        self.vel_R_sub = self.create_subscription(Float32, "wheel_command_right", self.R_callback,
                                                  qos_profile=qos.qos_profile_sensor_data)

        self.wheel_timer = self.create_timer(0.1, self.wheel_timer_callback)

        self.FL_speed = 0.0
        self.FR_speed = 0.0
        self.RL_speed = 0.0
        self.RR_speed = 0.0

    def writeSerial(self, x):
        ttl.write(bytes(x, 'utf-8'))

    def R_callback(self, vel_in):
        self.FR_speed = round(vel_in.data, 2)
        self.RR_speed = round(vel_in.data, 2)

    def L_callback(self, vel_in):
        self.FL_speed = round(vel_in.data, 2)
        self.RL_speed = round(vel_in.data, 2)

    def wheel_timer_callback(self):
        speed = "L"+str(self.FL_speed)+"R"+str(self.FR_speed) + \
            "L" + str(self.RL_speed)+"R" + str(self.RR_speed)+"/"
        self.writeSerial(speed)

    def mapper(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)


def main():
    rclpy.init()
    bu = bringup()

    rclpy.spin(bu)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
