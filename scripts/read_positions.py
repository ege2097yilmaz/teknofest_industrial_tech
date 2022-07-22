#!/usr/bin/env python3

import rospy
from paho.mqtt import client as mqtt_client
from teknofest_industrial_tech.msg import positions

broker = 'broker.emqx.io'
port = 1883
topic = "NCT/mqtt/channel/positions"

position = positions()
pub = rospy.Publisher('positions', positions, queue_size=10)

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        message1 = msg.payload.decode()

        message1 = message1.split(',')
        print(f"mesa {message1}")
        position.baslangic = str(message1[0])
        position.first_load = message1[1]
        position.first_unload = message1[2]
        position.second_load = message1[3]
        position.second_unload = message1[4]
        position.final = message1[0]
        pub.publish(position)

    client.subscribe(topic)
    client.on_message = on_message

def main_run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    rospy.init_node('positions_receiver')
    main_run()
    rospy.spin()