#!/usr/bin/env python3
import rospy
import traceback
from messages.msg import light as Light
from topics.sensor_topics import LIGHT_TOPIC


NODE_NAME = "light_sensor"
PUBLISHER_QUEUE_SIZE = 10


def init_node() -> None:
    rospy.init_node(NODE_NAME)


def init_publisher() -> None:
    global publisher
    publisher = rospy.Publisher(LIGHT_TOPIC, Light, queue_size = PUBLISHER_QUEUE_SIZE)


def publish_msg(msg: Light) -> None:
    publisher.publish(msg)


def read_sensor_loop() -> None:
    while not rospy.is_shutdown():
        try:
            user_input = input("enter light level :")
            value = float(user_input)
            msg = Light()
            msg.value = value
            publish_msg(msg)
        except:
            rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    init_node()
    init_publisher()
    read_sensor_loop()
