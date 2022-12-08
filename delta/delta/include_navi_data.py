#!/usr/bin/env python3
from datetime import datetime
import random
import paho.mqtt.client as mqtt
import socketio
import json
sio = socketio.Client()
host = "broker.hivemq.com"
port = 1883
topic_interval = "Robot5G_Nav_Interval"
topicStop = "Robot5G_Nav_Stop"
topicWaypoint = "Robot5G_Nav_Waypoint"
topicapture = "Robot5G_Nav_Capture"
stopic_control = 'control'
stopic_mode_navi = 'mode'
stopic_point_navi = 'Robot5G_Nav_Point'


def on_connect(self, client, userdata, rc):
    global mqtt_connect
    mqtt_connect = True
    self.subscribe(topicStop)
    self.subscribe(topicWaypoint)


def subMessage(client, userdata, msg):
    global nav_stop
    global nav_waypoint

    if (msg.topic == topicStop):
        msg_stop = msg.payload.decode("utf-8")
        msg_stop = json.loads(msg_stop)
        nav_stop = int(msg_stop.get("Stop"))

    if (msg.topic == topicWaypoint):
        msg_waypoint = msg.payload.decode("utf-8")
        msg_waypoint = json.loads(msg_waypoint)
        nav_waypoint = int(msg_waypoint.get("Waypoint"))


def on_publish(client, userdata, mid):
    countpub = format(mid)


@sio.event
def connect():
    global socket_connect
    socket_connect = True


@sio.event
def connect_error():
    print("The connection failed!")


@sio.event
def message(data):
    print('I received a message!')


@sio.on(stopic_control)
def on_message(data):
    global key_w
    global key_a
    global key_s
    global key_d
    key_w = int(data.get("W"))
    key_a = int(data.get("A"))
    key_s = int(data.get("S"))
    key_d = int(data.get("D"))


@sio.on(stopic_mode_navi)
def on_message(data):
    global get_mode
    get_mode = int(data.get("Mode"))


client = mqtt.Client()
client.connect(host)
client.on_connect = on_connect
client.on_message = subMessage
client.on_publish = on_publish

nav_stop = 0
nav_waypoint = 0
key_w = 0
key_a = 0
key_s = 0
key_d = 0
get_mode = 0

client.loop_start()

try:
    # sio.connect('https://delta-dee.tech/')
    sio.connect('https://robot5g-api.herokuapp.com')
except:
    sio.reconnection()


def convert_interval():
    global MSG_Sesor
    MSG_Sesor = json.dumps(
        {
            "Pzem_V": pzem_V,
            "Pzem_A": pzem_A,
            "Pzem_W": pzem_W,
            "Pzem_B": pzem_B,
            "Seq_pc": navi_seq
        })


def convert_point():
    global MSG_point
    MSG_point = json.dumps(
        {
            "Point": point
        })


def sendData(Volt, Watt, Amp, Batt, seq):
    global pzem_V
    global pzem_A
    global pzem_W
    global pzem_B
    global navi_seq
    pzem_V = Volt
    pzem_A = Watt
    pzem_W = Amp
    pzem_B = Batt
    navi_seq = seq
    convert_interval()
    client.publish(topic_interval, MSG_Sesor)


def send_navication(_point):
    global point
    point = _point
    convert_point()

    client.publish(topicapture, MSG_point)
