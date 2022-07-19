#!/usr/bin/env python3

import random, time
import rospy, tf
from paho.mqtt import client as mqtt_client

broker = 'broker.emqx.io'
port = 1883
topic = "NCT/mqtt/channel/odom"
time_interval = 1

# generate client ID with pub prefix randomly
client_id = f'NCT-{random.randint(0, 100)}'
username = 'NCT_robotic'
password = 'public-NCT'


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message

def main_run():
    client.loop_forever()

def publish(client, msg1):
    time.sleep(time_interval)
    result = client.publish(topic, msg1)
    status = result[0]
    if status == 0:
        print(f"Send `{msg1}`and to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")


def thread_func_listener_mqtt():
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            msg = str(trans)
            publish(client, msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


if __name__ == '__main__':
    rospy.init_node('tf_listener_nct_rob')

    listener = tf.TransformListener()

    client = connect_mqtt()

    rospy.loginfo("[INFO] threads are starting ...")
    thread_func_listener_mqtt()

    rate = rospy.Rate(10.0)

    