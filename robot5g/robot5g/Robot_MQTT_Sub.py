#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

import paho.mqtt.client as mqtt
import json

host = "broker.hivemq.com"
port = 1883
Control_value = 0.00
W = 0
A = 0
S = 0
D = 0
Mode_ = 0
Mode_Control_ = 0


class ModeControl(Node):

    def __init__(self):
        super().__init__("MQTT_Reader")
        self.mode_pub = self.create_publisher(Twist, "mode_Control", 10)
        self.stop_pub = self.create_publisher(Int16, "stop_Control", 10)
        # self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback,qos_profile=qos.qos_profile_sensor_data)

        self.stop_timer = self.create_timer(0.05, self.stop_timer_callback)
        self.mode_timer = self.create_timer(0.05, self.mode_timer_callback)
        self.Status = 0
        self.Mode_ = 0

    def on_connect(self, client, userdata, rc):
        self.subscribe("Robot5G/Web/Mode")
        self.subscribe("Robot5G/All/stop")
        self.subscribe("Robot5G/Web/Control")

    def on_message(self, client, msg):
        global Mode_, Mode_Control_
        if (msg.topic == "Robot5G/Web/Mode"):
            Mode_ = msg.payload.decode("utf-8")
            Mode_ = json.loads(Mode_)

            if (Mode_['Mode'] == True):
                #self.get_logger().info("Mode Manual")
                Mode_Control_ = 1

            elif (Mode_['Mode'] == False):
                #self.get_logger().info("Mode Automatics")
                Mode_Control_ = 0

        if (msg.topic == "Robot5G/Web/Control"):
            global Control_value, W, A, S, D
            Control_value = msg.payload.decode("utf-8")
            Control_value = json.loads(Control_value)
            W = Control_value['W']
            A = Control_value['A']
            S = Control_value['S']
            D = Control_value['D']

        if (msg.topic == "Robot5G/All/stop"):
            Stop = msg.payload.decode("utf-8")
            Stop_Value = json.loads(Stop)

    def on_publish(client, userdata, mid):
        countpub = format(mid)

    def joy_callback(self, data_in):
        pass

    def mode_timer_callback(self):
        self.Mode = Twist()
        self.Mode.linear.x = float(Mode_Control_)
        self.Mode.linear.y = float(W)
        self.Mode.linear.z = float(A)
        self.Mode.angular.x = float(S)
        self.Mode.angular.y = float(D)

        self.mode_pub.publish(self.Mode)

    def stop_timer_callback(self):
        self.Stop = Int16()
        self.Stop.data = 0

        self.stop_pub.publish(self.Stop)


client = mqtt.Client()
client.connect(host)
client.on_connect = ModeControl.on_connect
client.on_message = ModeControl.on_message
client.on_publish = ModeControl.on_publish
client.loop_start()


def main():
    rclpy.init()
    Mc = ModeControl()

    rclpy.spin(Mc)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
