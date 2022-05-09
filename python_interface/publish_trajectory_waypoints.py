# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import numpy as np  # Scientific computing library for Python
from python_interface import drone_interface
import rclpy
from rclpy import publisher
from rclpy import time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from time import sleep
import random
from drone_interface import DroneInterface

from std_msgs.msg import String
from as2_msgs.msg import TrajectoryWaypoints
from geometry_msgs.msg import PoseStamped


POSITION_THRESHOLD = 0.15


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


class MinimalPublisher():

    def __init__(self, drone_interface: DroneInterface):

        self.drone_interface = drone_interface
        topic_name = 'motion_reference/waypoints'

        if drone_interface.get_drone_id() == '/':
            topic_name = self.drone_interface.get_drone_id() + '/' + topic_name
        else:
            topic_name = '/' + self.drone_interface.get_drone_id() + '/' + topic_name

        print("Topic name : ", topic_name)

        self.publisher_ = self.drone_interface.create_publisher(
            TrajectoryWaypoints, topic_name, 10)
        # print('waiting')
        sleep(0.6)

    def land(self, speed=0.2, land_height=-1.0):
        land_points = self.drone_interface.get_position()
        land_points[2] = land_height
        print("Landing at : ", land_points)
        self.send_points([land_points], speed, TrajectoryWaypoints.KEEP_YAW)
        landed = False
        while landed == False:
            drone_height = self.drone_interface.get_position()[2]
            print("Drone height : ", drone_height)
            if drone_height < land_height:

                landed = True
            sleep(0.1)
        print("Landed")

    def take_off(self, speed=0.5, height=1.0):
        take_off_points = self.drone_interface.get_position()
        print("Taking off at : ", take_off_points)
        take_off_points[2] = height
        take_off_points[0] += 0.01
        take_off_points[1] += 0.01
        self.send_points([take_off_points], speed,
                         TrajectoryWaypoints.KEEP_YAW)
        took_off = False
        while took_off == False:
            position = self.drone_interface.get_position()
            print("Position : ", position)
            drone_height = position[2]

            print("Drone height : ", drone_height)
            if drone_height > height-POSITION_THRESHOLD  \
                    and drone_height < height + POSITION_THRESHOLD:
                took_off = True
            sleep(0.1)

    def send_points(self, point_list, speed, yaw_mode=TrajectoryWaypoints.KEEP_YAW, _yaw=0):
        msg = TrajectoryWaypoints()
        msg.header.stamp = self.drone_interface.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        # msg.yaw_mode = TrajectoryWaypoints.KEEP_YAW
        msg.yaw_mode = yaw_mode
        poses = []
        for point in point_list:
            pose = PoseStamped()
            x, y, z = point
            yaw_def = _yaw
            orientation_quaternion = get_quaternion_from_euler(0, 0, yaw_def)
            pose.pose.position.x = (float)(x)
            pose.pose.position.y = (float)(y)
            pose.pose.position.z = (float)(z)
            pose.pose.orientation.w = 1.0
            # pose.pose.orientation.x = orientation_quaternion[0]
            # pose.pose.orientation.y = orientation_quaternion[1]
            # pose.pose.orientation.z = orientation_quaternion[2]
            # pose.pose.orientation.w = orientation_quaternion[3]
            poses.append(pose)
        msg.poses = poses
        msg.max_speed = (float)(speed)
        print("Sending message : ", point_list)
        self.publisher_.publish(msg)
        sleep(1.0)


def main(args=None):
    rclpy.init(args=args)

    # drone_interface = DroneInterface("drone0")
    drone_interface = DroneInterface("drone_sim_dps_0")
    minimal_publisher = MinimalPublisher(drone_interface)

    takeoff = 1
    land = 0
    # land = not takeoff
    rth = 0
    lap = 0

    if takeoff:
        minimal_publisher.take_off(speed=0.3, height=1.5)

    elif land:

        minimal_publisher.land()

    # elif rth:

    #     speed = 10

    #     point_lists = [[0, 0, 1]]
    #     minimal_publisher.send_points(
    #         point_lists, speed, TrajectoryWaypoints.KEEP_YAW)

    # else:
    #     if lap == 1:
    #         speed = 5.0
    #         point_lists = [[1, 1, 1.5],
    #                        [-1, 1, 1.5],
    #                        [-1, -1, 1.5],
    #                        [1, -1, 1.5],
    #                        [0, 0, 1.5]]
    #         # point_lists = [[20, -20, 5]]

    #         # minimal_publisher.send_points(point_lists,speed,TrajectoryWaypoints.PATH_FACING)
    #         minimal_publisher.send_points(
    #             point_lists, speed, TrajectoryWaypoints.PATH_FACING)

    else:
        if lap == 1:
            speed = 0.5
            point_lists = [[0.0, 0, 1.5]]
            yaw_to_send = 0.4

            minimal_publisher.send_points(
                point_lists, speed, TrajectoryWaypoints.PATH_FACING)
            # minimal_publisher.send_points(
            #     point_lists, speed, TrajectoryWaypoints.KEEP_YAW)

        # if lap == 2:

        #     speed = 5.0
        #     point_lists = [[7, 3, 1.5],
        #                    [7, -2, 2],
        #                    [0, 0, 2],
        #                    [-7, 3, 1.5],
        #                    [-7, -2, 2],
        #                    [0, 0, 1.5]]

        #     minimal_publisher.send_points(
        #         point_lists, speed, TrajectoryWaypoints.KEEP_YAW)

        # if lap == 3:
        #     speed = 7.0
        #     point_lists = [[7, 3, 1.5],
        #                    [7, -2, 2],
        #                    [0, 0, 2],
        #                    [-7, 3, 1.5],
        #                    [-7, -2, 2],
        #                    [0, 0, 1.5]]

        #     minimal_publisher.send_points(
        #         point_lists, speed, TrajectoryWaypoints.KEEP_YAW)

    minimal_publisher.drone_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
