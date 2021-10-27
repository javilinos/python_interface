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
from aerostack2_msgs.msg import TrajectoryWaypoints
from geometry_msgs.msg import PoseStamped


POSITION_THRESHOLD = 0.15

class MinimalPublisher():

    def __init__(self, drone_interface: DroneInterface):
        
        self.drone_interface = drone_interface
        topic_name = 'motion_reference/waypoints'
        
        if drone_interface.get_drone_id() == '/':
            topic_name = self.drone_interface.get_drone_id() + '/' + topic_name
        else:
            topic_name = '/' + self.drone_interface.get_drone_id() + '/' + topic_name

        print("Topic name : ", topic_name)

        self.publisher_ = self.drone_interface.create_publisher(TrajectoryWaypoints, topic_name, 10)
        # print('waiting')
        sleep(0.6)

    def land(self,speed = 0.3,land_height = -1.0):
        land_points = self.drone_interface.get_position()
        land_points[2] = land_height
        print("Landing at : ", land_points)
        self.send_points([land_points],speed,TrajectoryWaypoints.KEEP_YAW)
        landed = False
        while landed == False:
            drone_height=self.drone_interface.get_position()[2] 
            print("Drone height : ", drone_height)
            if drone_height < land_height :  

                landed = True
            sleep(0.1)
        print("Landed")
    
    def take_off(self,speed = 0.5,height=1.0):
        take_off_points = self.drone_interface.get_position()
        print("Taking off at : ", take_off_points)
        take_off_points[2] = height
        self.send_points([take_off_points],speed,TrajectoryWaypoints.KEEP_YAW)
        took_off = False
        while took_off == False:
            position = self.drone_interface.get_position()
            print("Position : ", position)
            drone_height=position[2]

            print("Drone height : ", drone_height)
            if drone_height > height-POSITION_THRESHOLD  \
                and drone_height  < height + POSITION_THRESHOLD :
                took_off = True
            sleep(0.1)


    def send_points(self,point_list,speed,yaw_mode = TrajectoryWaypoints.KEEP_YAW):
        msg = TrajectoryWaypoints()
        msg.header.stamp = self.drone_interface.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        # msg.yaw_mode = TrajectoryWaypoints.KEEP_YAW
        msg.yaw_mode = yaw_mode
        poses = []
        for point in point_list:
            pose = PoseStamped()
            x,y,z = point
            pose.pose.position.x = (float)(x)
            pose.pose.position.y = (float)(y)
            pose.pose.position.z = (float)(z)
            pose.pose.orientation.w=1.0
            poses.append(pose)
        msg.poses = poses
        msg.max_speed = (float)(speed)
        print("Sending message : ", point_list)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    drone_interface = DroneInterface("drone_sim_0")
    minimal_publisher = MinimalPublisher(drone_interface)

    
    minimal_publisher.take_off()
    # minimal_publisher.land()
    
    # point_lists = [[3,2,5],[-3,4,3],[3,-4,4]]

    # minimal_publisher.send_points(point_lists,2.5,TrajectoryWaypoints.PATH_FACING)
 
    
    minimal_publisher.drone_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()