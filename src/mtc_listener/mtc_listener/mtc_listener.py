# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
# package for the transformation
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# import pandas as pd
# import numpy as np
import csv
import os


# from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('mtc_listener')



        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.timer = self.create_timer(0.10, self.on_timer)

    def on_timer(self):
 
        from_frame_rel = "panda_link0"
        to_frame_rel = 'panda_hand'


        self.file_name = "received_data.csv"
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.get_logger().info('logger_working')
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.y
            time = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9

            with open(self.file_name, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([x, y, z, time])

            self.get_logger().info('Data saved')

            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

 

def main():
    print("running node")
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
