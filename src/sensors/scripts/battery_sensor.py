#!/usr/bin/env python3
import rospy
import traceback
from messages.msg import battery as Batttery
from topics.sensor_topics import BATTERY_TOPIC


NODE_NAME = "battery_sensor"
PUBLISHER_QUEUE_SIZE = 10


def init_node() -> None:
    rospy.init_node(NODE_NAME)


def init_publisher() -> None:
    global publisher
    publisher = rospy.Publisher(BATTERY_TOPIC, Batttery, queue_size = PUBLISHER_QUEUE_SIZE)


def publish_msg(msg: Batttery) -> None:
    publisher.publish(msg)


def read_sensor_loop() -> None:
    while not rospy.is_shutdown():
        try:
            user_input = input("enter battery level :")
            value = float(user_input)
            msg = Batttery()
            msg.value = value
            publish_msg(msg)
        except:
            rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    init_node()
    init_publisher()
    read_sensor_loop()
