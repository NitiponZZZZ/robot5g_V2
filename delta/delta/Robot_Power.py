#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos
import serial
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
import json
from pymodbus.client.serial import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder


client = ModbusClient(method='RTU', port='/dev/pzem',
                      timeout=1, baudrate=9600, stopbits=2, bytesize=8, parity='N')


class feedback(Node):
    def __init__(self):
        super().__init__("robot_power")

        self.pzem_pub = self.create_publisher(
            Twist, "power_status", 10)

        self.pzem_timer = self.create_timer(5, self.pzem_timer_callback)

        self.pzcurrent = 0.00
        self.pzvoltage = 0.00
        self.pzpower = 0.00
        self.pzpercent = 0.00

    def mapper(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

    def pzem_timer_callback(self):
        try:
            data = client.read_input_registers(0, 6)
            dataDecoder = BinaryPayloadDecoder.fromRegisters(
                data.registers, byteorder=Endian.Big, wordorder=Endian.Little)
            self.pzvoltage = dataDecoder.decode_16bit_int() / 100        # adress 0x0000

            # 0x0001 *0.5 because power bridge range is higher
            self.pzcurrent = (dataDecoder.decode_16bit_int() / 100)*0.5

            self.pzpower = self.pzvoltage * self.pzcurrent               # 0x0002, 0x0003

            self.pzpercent = self.mapper(self.pzvoltage, 36, 42, 0, 100)

            power_status = Twist()
            power_status.linear.x = float(self.pzvoltage)
            power_status.linear.y = float(self.pzcurrent)
            power_status.linear.z = float(self.pzpower)
            power_status.angular.x = float(self.pzpercent)
            self.pzem_pub.publish(power_status)
        except json.JSONDecodeError as e:
            if not client.connect():
                print("unable to connect")
                exit(-1)


def main():
    rclpy.init()
    fb = feedback()

    rclpy.spin(fb)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
