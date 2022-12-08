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
        self.Seq_Control = 0
        self.Seq_Navigation = 0

        self.Pzem_Voltage = 0.0
        self.Pzem_Amp = 0.0
        self.Pzem_Watt = 0.0
        self.Percent_Battery = 0.0

    def pzem_callback(self, pz_in):
        self.Pzem_Voltage = round(pz_in.linear.x, 2)
        self.Pzem_Amp = round(pz_in.linear.y, 2)
        self.Pzem_Watt = round(pz_in.linear.z, 2)
        self.Percent_Battery = round(pz_in.angular.x, 2)

        # vPow = 5.0
        # r1 = 100000
        # r2 = 10000

        # v = self.Pzem_Voltage
        # v2 = v / (r2 / (r1 + r2))

        # top = 42
        # bottom = 36
        # range = top - bottom
        # rangeVolts = v2 - bottom
        # percent = (rangeVolts / range) * 100

        # if (percent > 100){
        #     percent = 100
        # }
        # if (percent < 0) {
        #     percent = 0
        # }
        # self.Percent_Battery = round(percent, 2)

    def robotcheck_callback(self, robot_in):
        self.Seq_Control = int(robot_in.x)
        self.Seq_Navigation = int(robot_in.y)

    # def sendMQTT_timer_callback(self):
    #     self.Imu_X = random.randint(0, 1)
    #     self.Imu_Y = random.randint(0, 1)
    #     self.Imu_Z = random.randint(0, 1)

    def sendMQTT2_timer_callback(self):
        navi_data.sendData(self.Pzem_Voltage, self.Pzem_Amp,
                           self.Pzem_Watt, self.Percent_Battery, self.Seq_Navigation)


def main():
    rclpy.init()
    MP = MQPUB()

    rclpy.spin(MP)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
