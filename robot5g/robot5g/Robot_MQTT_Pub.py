#!/usr/bin/env python3
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import paho.mqtt.client as mqtt
import json
import random

host = "broker.hivemq.com"
port = 1883


class MQPUB(Node):

    def __init__(self):
        super().__init__("MQTT_Sender")
        self.pzem_sub = self.create_subscription(Twist, "power_status", self.pzem_callback,
                                                 qos_profile=qos.qos_profile_sensor_data)
        self.sensors_sub = self.create_subscription(Twist, "sensor_status", self.sensors_callback,
                                                    qos_profile=qos.qos_profile_sensor_data)
        self.odrive_sub = self.create_subscription(Twist, "sensor_status", self.odrive_callback,
                                                   qos_profile=qos.qos_profile_sensor_data)
        self.robotcheck_sub = self.create_subscription(Vector3, "robotcheck_status", self.robotcheck_callback,
                                                       qos_profile=qos.qos_profile_sensor_data)
        self.wheel_vel_sub = self.create_subscription(Twist, "wheel_vel", self.wheel_vel_callback,
                                                      qos_profile=qos.qos_profile_sensor_data)

        self.sendMQTT_timer = self.create_timer(
            1, self.sendMQTT_timer_callback)
        self.sendMQTT2_timer = self.create_timer(
            1, self.sendMQTT2_timer_callback)
        self.Voltage_Odrive1 = 0.0
        self.Voltage_Odrive2 = 0.0
        self.Velocity_FL = 0.0
        self.Velocity_FR = 0.0
        self.Velocity_RL = 0.0
        self.Velocity_RR = 0.0
        self.UltraF = 0.0
        self.UltraR = 0.0
        self.Speed_Robot = 0.0
        self.Imu_X = 0.0
        self.Imu_Y = 0.0
        self.Imu_Z = 0.0
        self.Seq_Control = 0.0
        self.Seq_Navigation = 0.0
        self.Temp_Batt = 0.0
        self.Humid_Batt = 0.0
        self.Temp_Robot = 0.0
        self.Humid_Robot = 0.0
        self.Pzem_Voltage = 0.0
        self.Pzem_Amp = 0.0
        self.Pzem_Watt = 0.0
        self.Percent_Battery = 0.0

    def on_connect(self, client, userdata, rc):
        pass

    def on_message(self, client, msg):
        pass

    def on_publish(client, userdata, mid):
        countpub = format(mid)

    def joy_callback(self, data_in):
        pass

    def convert_Json(self):
        global MQTT_MSG
        MQTT_MSG = json.dumps(
            {
                "VO1": self.Voltage_Odrive1,
                "VO2": self.Voltage_Odrive2,
                "VFL": self.Velocity_FL,
                "VFR": self.Velocity_FR,
                "VRL": self.Velocity_RL,
                "VRR": self.Velocity_RR,
                "USF":  self.UltraF,
                "USR": self.UltraR,
                "SR": self.Speed_Robot,
                "Imu_X": self.Imu_X,
                "Imu_Y": self.Imu_Y,
                "Imu_Z": self.Imu_Z,
                "Seq_u": self.Seq_Control,
                "Seq_pc": self.Seq_Navigation,

            })

    def convert_Json2(self):
        global MQTT_MSG2
        MQTT_MSG2 = json.dumps(
            {
                "TB": self.Temp_Batt,
                "HB": self.Humid_Batt,
                "TR": self.Temp_Robot,
                "HR": self.Humid_Robot,
                "Pzem_V": self.Pzem_Voltage,
                "Pzem_A": self.Pzem_Amp,
                "Pzem_W": self.Pzem_Watt,
                "Pzem_B": self.Percent_Battery,

            })

    def wheel_vel_callback(self, vell_in):
        self.Velocity_FL = round(vell_in.linear.x, 2)
        self.Velocity_FR = round(vell_in.linear.y, 2)
        self.Velocity_RL = round(vell_in.linear.z, 2)
        self.Velocity_RR = round(vell_in.angular.x, 2)

    def pzem_callback(self, pz_in):
        self.Pzem_Voltage = round(pz_in.linear.x, 2)
        self.Pzem_Amp = round(pz_in.linear.y, 2)
        self.Pzem_Watt = round(pz_in.linear.z, 2)
        self.Percent_Battery = round(pz_in.angular.x, 2)

    def sensors_callback(self, sen_in):
        self.UltraF = round(sen_in.linear.x, 2)
        self.UltraR = round(sen_in.linear.y, 2)
        self.Temp_Batt = round(sen_in.linear.z, 2)
        self.Humid_Batt = round(sen_in.angular.x, 2)
        self.Temp_Robot = round(sen_in.angular.y, 2)
        self.Humid_Robot = round(sen_in.angular.z, 2)

    def odrive_callback(self, odv_in):
        self.Voltage_Odrive1 = round(odv_in.linear.x, 2)
        self.Voltage_Odrive2 = round(odv_in.linear.y, 2)
        self.Velocity_FL = round(odv_in.linear.z, 2)
        self.Velocity_FR = round(odv_in.angular.x, 2)
        self.Velocity_RL = round(odv_in.angular.y, 2)
        self.Velocity_RR = round(odv_in.angular.z, 2)

    def robotcheck_callback(self, robot_in):
        self.Speed_Robot = round(robot_in.linear.x, 2)
        self.Seq_Control = round(robot_in.linear.y, 2)
        self.Seq_Navigation = round(robot_in.linear.z, 2)

    def sendMQTT_timer_callback(self):
        self.Imu_X = random.randint(0, 1)
        self.Imu_Y = random.randint(0, 1)
        self.Imu_Z = random.randint(0, 1)

        self.convert_Json()
        client.publish("Robot5G/MPC/Interval", MQTT_MSG)

    def sendMQTT2_timer_callback(self):

        self.convert_Json2()
        client.publish("Robot5G/MPC/sensor", MQTT_MSG2)


client = mqtt.Client()
client.connect(host)
client.on_connect = MQPUB.on_connect
client.on_message = MQPUB.on_message
client.on_publish = MQPUB.on_publish
client.loop_start()


def main():
    rclpy.init()
    MP = MQPUB()

    rclpy.spin(MP)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
