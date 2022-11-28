#!/usr/bin/env python3
import random

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos
from rclpy.node import Node

from . import include_navi_data as navi_data


class MQPUB(Node):

    def __init__(self):
        super().__init__("MQTT_Sender")
        self.pzem_sub = self.create_subscription(Twist, "power_status", self.pzem_callback,
                                                 qos_profile=qos.qos_profile_sensor_data)
        self.robotcheck_sub = self.create_subscription(Vector3, "robotcheck_status", self.robotcheck_callback,
                                                       qos_profile=qos.qos_profile_sensor_data)

        # self.sendMQTT_timer = self.create_timer(
        #     1, self.sendMQTT_timer_callback)

        self.sendMQTT2_timer = self.create_timer(
            1, self.sendMQTT2_timer_callback)

        self.Speed_Robot = 0.0
        self.Imu_X = 0.0
        self.Imu_Y = 0.0
        self.Imu_Z = 0.0
        self.Seq_Control = 0.0
        self.Seq_Navigation = 0.0

        self.Pzem_Voltage = 0.0
        self.Pzem_Amp = 0.0
        self.Pzem_Watt = 0.0
        self.Percent_Battery = 0.0

    def pzem_callback(self, pz_in):
        self.Pzem_Voltage = round(pz_in.linear.x, 2)
        self.Pzem_Amp = round(pz_in.linear.y, 2)
        self.Pzem_Watt = round(pz_in.linear.z, 2)
        self.Percent_Battery = round(pz_in.angular.x, 2)

    def robotcheck_callback(self, robot_in):
        self.Seq_Control = round(robot_in.x, 2)
        self.Seq_Navigation = round(robot_in.y, 2)

    # def sendMQTT_timer_callback(self):
    #     self.Imu_X = random.randint(0, 1)
    #     self.Imu_Y = random.randint(0, 1)
    #     self.Imu_Z = random.randint(0, 1)

    def sendMQTT2_timer_callback(self):
        navi_data.sendData(self.Pzem_Voltage, self.Pzem_Watt,
                           self.Pzem_Amp, self.Percent_Battery, self.Seq_Navigation)
        pass


def main():
    rclpy.init()
    MP = MQPUB()

    rclpy.spin(MP)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
