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

from rmf_lift_msgs.msg import LiftRequest


def main(argv=sys.argv):
    """
    Example lift request:
    - lift_name: magni123
    - session_id: amr001
    - request_type: agv
    - destination_floor: L1
    - door_state: closed
    """

    default_lift_name = "LIFT-001"
    default_session_id = "amr001"
    default_request_type = "agv"
    default_destination_floor = "L2"
    default_door_state = "closed"
    default_topic_name = "/adapter_lift_requests"

    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--lift-name", default=default_lift_name)
    parser.add_argument("-s", "--session-id", default=default_session_id)
    parser.add_argument("-r", "--request-type", default=default_request_type)
    parser.add_argument("-f", "--destination-floor", default=default_destination_floor)
    parser.add_argument("-d", "--door-state", default=default_door_state)
    parser.add_argument("-t", "--topic-name", default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print("lift_name: {}".format(args.lift_name))
    print("session_id: {}".format(args.session_id))
    print("request_type: {}".format(args.request_type))
    print("destination_floor: {}".format(args.destination_floor))
    print("door_state: {}".format(args.door_state))
    print("topic_name: {}".format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node("send_adapter_lift_requests_node")
    pub = node.create_publisher(LiftRequest, args.topic_name, 10)

    msg = LiftRequest()
    msg.request_time = node.get_clock().now().to_msg()
    msg.lift_name = args.lift_name
    msg.session_id = args.session_id
    msg.destination_floor = args.destination_floor

    if args.request_type == "mode":
        print("Please insert desired request_type: end, agv or human")
        return
    elif args.request_type == "end":
        msg.request_type = LiftRequest.REQUEST_END_SESSION
    elif args.request_type == "agv":
        msg.request_type = LiftRequest.REQUEST_AGV_MODE
    elif args.request_type == "human":
        msg.request_type = LiftRequest.REQUEST_HUMAN_MODE
    else:
        print("unrecognized request_type, only use end, agv or human please")
        return

    if args.door_state == "state":
        print("Please insert desired door_state: closed or open")
        return
    elif args.door_state == "closed":
        msg.door_state = LiftRequest.DOOR_CLOSED
    elif args.door_state == "open":
        msg.door_state = LiftRequest.DOOR_OPEN
    else:
        print("unrecognized door_state, only use closed or open please")
        return

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("all done!")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
