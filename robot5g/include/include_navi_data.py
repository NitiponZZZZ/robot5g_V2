import paho.mqtt.client as mqtt
import socketio
import json
sio = socketio.Client()
import random
from datetime import datetime
host = "broker.hivemq.com"
port = 1883
topic_interval= "Robot5G_Nav_Interval"
topicStop = "Robot5G_Nav_Stop"
topicapture = "Robot5G_Nav_Capture"
stopic_control ='control'
stopic_mode_navi ='Robot5G_Nav_Mode'
stopic_point_navi ='Robot5G_Nav_Point'


def on_connect(self, client, userdata, rc):
    global mqtt_connect
    mqtt_connect = True
    self.subscribe(topicStop)
    #mqtt_connect = True
    
  
    

           
def subMessage(client, userdata,msg):
    global nav_stop

    if  (msg.topic == topicStop):  
        msg_stop = msg.payload.decode("utf-8")
        msg_stop = json.loads(msg_stop)
        nav_stop = str(msg_stop.get("Stop"))
        #print("data : " + str(msg_stop.get("Stop")))
       
    #     Mode_ = msg.payload.decode("utf-8")
    #     Mode_ = json.loads(Mode_)

    


def on_publish(client, userdata, mid):
    countpub = format(mid)



@sio.event
def connect():
    global socket_connect
    socket_connect = True
    #print("I'm connected!")



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
    key_w   =str(data.get("W"))
    key_a   =str(data.get("A"))
    key_s   =str(data.get("S"))
    key_d   =str(data.get("D"))
   #print('data receive :'+ str(data))

@sio.on(stopic_mode_navi)
def on_message(data):
     global get_mode
     get_mode = str(data.get("Mode"))




@sio.on('price')
def on_message(data):
    print('Price Data ', data)


client = mqtt.Client()
client.connect(host)
client.on_connect = on_connect
client.on_message = subMessage
client.on_publish = on_publish
nav_stop =0
key_w   = 0
key_a   = 0
key_s   = 0
key_d   = 0
client.loop_start()


try :
    sio.connect('https://robot5g-client.herokuapp.com')
except :
    sio.reconnection()


def  convert_interval  ():
    global MSG_Sesor
    MSG_Sesor=json.dumps(
        {
            "Pzem_V" : pzem_V,
            "Pzem_A" : pzem_A,
            "Pzem_W" : pzem_W,
            "Pzem_B" : pzem_B,
            "Seq_pc" : navi_seq
        })



def  convert_point  ():
    global MSG_point
    MSG_point=json.dumps(
        {
            "Point" : point
        })


def sendData(Volt,Watt,Amp,Batt,seq):
    global pzem_V
    global pzem_A
    global pzem_W
    global pzem_B  
    global navi_seq              
    pzem_V =Volt
    pzem_A =Watt
    pzem_W =Amp
    pzem_B =Batt
    navi_seq = seq
    convert_interval ()
    client.publish(topic_interval,MSG_Sesor)


def send_navication (_point):
    global point
    point = _point
    convert_point ()
    
    client.publish(topicapture,MSG_point)



