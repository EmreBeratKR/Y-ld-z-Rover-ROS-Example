#!/usr/bin/env python3
import rospy
import traceback
from messages.msg import temperature as Temperature
from topics.sensor_topics import TEMPERATURE_TOPIC


NODE_NAME = "temperature_sensor"
PUBLISHER_QUEUE_SIZE = 10


def init_node() -> None:
    rospy.init_node(NODE_NAME)


def init_publisher() -> None:
    global publisher
    publisher = rospy.Publisher(TEMPERATURE_TOPIC, Temperature, queue_size = PUBLISHER_QUEUE_SIZE)


def publish_msg(msg: Temperature) -> None:
    publisher.publish(msg)


def read_sensor_loop() -> None:
    while not rospy.is_shutdown():
        try:
            user_input = input("enter temperature message :")
            split = user_input.split(' ')
            value = float(split[0])
            unit = split[1]
            msg = Temperature()
            msg.value = value
            msg.unit.data = unit
            publish_msg(msg)
        except:
            rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    init_node()
    init_publisher()
    read_sensor_loop()
