#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import argparse

import rclpy
from rclpy.node import Node

from ldm_fleet_msgs.msg import RegisterRequest


def main(argv=sys.argv):
    """
    Example register request:
    - request_id: 6tyghb4eduj1fyd
    - device_name: magni123
    - device_type: lift
    - register_mode: released
    """

    default_request_id = "576y13ewgyffeijuais"
    default_device_name = "LIFT-001"
    default_device_type = "lift"
    default_register_mode = "signed"
    default_topic_name = "/register_ldm_requests"

    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--device-name", default=default_device_name)
    parser.add_argument("-d", "--device-type", default=default_device_type)
    parser.add_argument("-r", "--register-mode", default=default_register_mode)
    parser.add_argument("-i", "--request-id", default=default_request_id)
    parser.add_argument("-t", "--topic-name", default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print("device_name: {}".format(args.device_name))
    print("device_type: {}".format(args.device_type))
    print("register_mode: {}".format(args.register_mode))
    print("request_id: {}".format(args.request_id))
    print("topic_name: {}".format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node("send_register_request_node")
    pub = node.create_publisher(RegisterRequest, args.topic_name, 10)

    msg = RegisterRequest()
    msg.device_name = args.device_name
    msg.request_id = args.request_id

    if args.device_type == "door":
        msg.device_type = RegisterRequest.DEVICE_DOOR
    elif args.device_type == "lift":
        msg.device_type = RegisterRequest.DEVICE_LIFT
    else:
        print("unrecognized device_type, only use door or lift please")
        return

    if args.register_mode == "released":
        msg.register_mode = RegisterRequest.REGISTER_RELEASED
    elif args.register_mode == "signed":
        msg.register_mode = RegisterRequest.REGISTER_SIGNED
    else:
        print("unrecognized register_mode, only use released or signed please")
        return

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("all done!")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
