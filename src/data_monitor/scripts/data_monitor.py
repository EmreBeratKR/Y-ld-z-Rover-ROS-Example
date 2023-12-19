#!/usr/bin/env python3
import customtkinter as ctk
import rospy
from messages.msg import temperature as Temperature
from messages.msg import battery as Batttery
from messages.msg import light as Light
from topics.sensor_topics import TEMPERATURE_TOPIC, BATTERY_TOPIC, LIGHT_TOPIC


NODE_NAME = "data_monitor"

WINDOW_TITLE = "Rover Data Monitor"
WINDOW_WIDTH = 600
WINDOW_HEIGHT = 300


def init_node() -> None:
    rospy.init_node(NODE_NAME)


def subscribe_required_topics() -> None:
    rospy.Subscriber(TEMPERATURE_TOPIC, Temperature, on_temperature_topic)
    rospy.Subscriber(BATTERY_TOPIC, Batttery, on_battery_topic)
    rospy.Subscriber(LIGHT_TOPIC, Light, on_light_topic)


def create_window() -> None:
    global window
    window = ctk.CTk()
    window.title(WINDOW_TITLE)
    window.resizable(width=False, height=False)
    window.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")


def loop_window() -> None:
    window.mainloop()


def create_banner() -> None:
    title = ctk.CTkLabel(window, text=WINDOW_TITLE, font=ctk.CTkFont(size=30, weight="bold"))
    title.pack(pady=(20, 0))


def create_monitor() -> None:
    global battery_level
    global battery_percent
    global light_level
    global temperature_level

    frame = ctk.CTkFrame(window)
    frame.pack_configure(padx=40, pady=(40, 40))
    frame.pack()

    battery_frame = ctk.CTkFrame(frame, width=200, height=40)
    battery_frame.pack_configure(anchor="w", pady=(10, 10), padx=(20, 20))
    battery_frame.pack()
    battery_label = ctk.CTkLabel(battery_frame, text="Battery Level")
    battery_label.pack_configure(side="left", padx=(20, 20))
    battery_label.pack()
    battery_level = ctk.CTkProgressBar(battery_frame)
    battery_level.pack_configure(side="left")
    battery_level.pack()
    battery_percent = ctk.CTkLabel(battery_frame)
    battery_percent.pack_configure(side="right", padx=(20, 20))
    battery_percent.pack()
    set_battery_level(0)

    light_frame = ctk.CTkFrame(frame, width=100, height=20)
    light_frame.pack_configure(anchor="w", padx=(20, 20))
    light_frame.pack()
    light_label = ctk.CTkLabel(light_frame, text="Light Level")
    light_label.pack_configure(side="left", padx=(20, 20))
    light_label.pack()
    light_level = ctk.CTkLabel(light_frame)
    light_level.pack_configure(side="right", padx=(20, 20))
    light_level.pack()
    set_light_level(0)

    temperature_frame = ctk.CTkFrame(frame, width=100, height=20)
    temperature_frame.pack_configure(anchor="w", pady=(10, 10), padx=(20, 20))
    temperature_frame.pack()
    temperature_label = ctk.CTkLabel(temperature_frame, text="Temperature")
    temperature_label.pack_configure(side="left", padx=(20, 20))
    temperature_label.pack()
    temperature_level = ctk.CTkLabel(temperature_frame)
    temperature_level.pack_configure(side="right", padx=(20, 20))
    temperature_level.pack()
    set_temperature(0, "⁰C")


def set_battery_level(value: float) -> None:
    t = value / 100.0
    color = lerp_color((232, 74, 46), (102, 232, 46), t)
    hex_color = RGB2HEX(color)
    battery_level.set(t)
    battery_level.configure(progress_color=hex_color)
    battery_percent.configure(text=f"%{'%.1f' % value}")


def set_light_level(value: float) -> None:
    light_level.configure(text=f"{'%.0f' % value} lux")


def set_temperature(value: float, unit: str) -> None:
    temperature_level.configure(text=f"{'%.1f' % value} {unit}")


def on_temperature_topic(msg: Temperature) -> None:
    rospy.loginfo(msg)
    value = msg.value
    unit = msg.unit.data
    set_temperature(value, unit)


def on_battery_topic(msg: Batttery) -> None:
    rospy.loginfo(msg)
    battery_level = msg.value
    set_battery_level(battery_level)


def on_light_topic(msg: Light) -> None:
    rospy.loginfo(msg)
    light_level = msg.value
    set_light_level(light_level)


def lerp_color(color_a: (int, int, int), color_b: (int, int, int), t: float) -> (int, int, int):
    r = lerp(color_a[0], color_b[0], t)
    g = lerp(color_a[1], color_b[1], t)
    b = lerp(color_a[2], color_b[2], t)
    return (int(r), int(g), int(b))


def RGB2HEX(color: (int, int, int)) -> str:
    return '#%02x%02x%02x' % color


def lerp(a: float, b: float, t: float) -> float:
    return (1 - t) * a + t * b


if __name__ == "__main__":
    init_node()
    subscribe_required_topics()
    create_window()
    create_banner()
    create_monitor()
    loop_window()
