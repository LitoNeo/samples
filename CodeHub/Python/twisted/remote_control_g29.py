#!/usr/bin/env python
# -*-coding:utf-8-*-

import rospy
from sensor_msgs.msg import Joy
import json
import time
from Queue import Queue, Empty
from connector import Connector


STEER = 0
SPEED = 1
BRAKE = 2
# LIGHT_LEFT = 3
# LIGHT_RIGHT = 4
# VOICE_UP = 5
# VOICE_DOWN = 6

# data{
# 	"version": "0.3", # 向后兼容
# 	"type": REMOTE_CONTROL , # REMOTE_CONTROL = 0x25
# 	"ack": int, # 响应标识，取值参考ack字段说明
# 	#    请求:254
# 	#    回复:
# 	#      成功: 0
# 	#      失败: 1
# 	"requestId": int, # 请求唯一标识
# 	"vin": "00000000...", # 车辆唯一标识, 17位
# 	"data": {} # 协议内容字段，根据type类型确定字段内容。
# }

"""
# REMOTE_CONTROL = 0x25

"timestamp" = rospy.Time.now().to_time() * 1000
"msg" = {
    "steer":(float),
    "speed":(float),
    "brake":(float)
}
"""


class ControlMessage:
    def __init__(self):
        self.steer = 0.
        self.speed = 0.
        self.brake = 0.


class App:
    def __init__(self):
        rospy.init_node("remote_control_g29", log_level=rospy.DEBUG)
        self._message = ControlMessage()
        self.remote_vin = None
        self.host = None
        self.port = None
        self.send_rate = None
        self.event_bus = Queue()
        self.client = None

    def run(self):
        self.init_params()
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.event_loop()
        rospy.spin()

    def init_params(self):
        self.remote_vin = rospy.get_param("~remote_vin")
        self.host = rospy.get_param("~host")
        self.port = rospy.get_param("~port")
        self.send_rate = rospy.get_param("~send_rate")
        if self.remote_vin is not None and self.host is not None and self.port is not None:
            self.client = Connector(self.event_bus, self.host, self.port)
        else:
            rospy.ERROR("[remote_control_g29]: config unset, exit.")
            exit(1)

    def joy_callback(self, msg):
        assert isinstance(msg, Joy)
        self._message.steer = Joy.axes[STEER]
        self._message.speed = Joy.axes[SPEED]
        self._message.brake = Joy.axes[BRAKE]

    def event_loop(self):
        hz = 1.0 / self.send_rate
        while True:
            json_msg = self.create_json_message()
            self.client.write(json_msg)
            time.sleep(hz)

    def create_json_message(self):
        data = self.create_dict_message_base()
        data["data"] = self.create_dict_control_message_base()
        return json.dumps(data)

    def create_dict_message_base(self):
        data = {
            "version": "0.3",  # 向后兼容
            "type": 0x25,  # REMOTE_CONTROL = 0x25
            "ack": 254,  # 响应标识，取值参考ack字段说明
            "requestId": 0,  # 请求唯一标识
            "vin": self.remote_vin,  # 车辆唯一标识, 17位
            "data": {}  # 协议内容字段，根据type类型确定字段内容。
        }
        return data

    def create_dict_control_message_base(self):
        data = {
            "timestamp": rospy.Time.now().to_time() * 1000,
            "msg": {
                "steer": self._message.steer,
                "speed": self._message.speed,
                "brake": self._message.brake
            }
        }
        return data


if __name__ == '__main__':
    app = App()
    try:
        app.run()
    except Exception as e:
        print(e)
