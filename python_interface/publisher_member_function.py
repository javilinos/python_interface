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

import rclpy
from rclpy import publisher
from rclpy.node import Node

from std_msgs.msg import String
from aerostack2_msgs.msg import TrajectoryWaypoints
from geometry_msgs.msg import PoseStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TrajectoryWaypoints, '/drone0/motion_reference/waypoints', 10)

    def send_points(self,point_list,speed):
        msg = TrajectoryWaypoints()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.yaw_mode = TrajectoryWaypoints.KEEP_YAW
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

    minimal_publisher = MinimalPublisher()

    point_lists = [[0,0,1]]
    minimal_publisher.send_points(point_lists,0.5)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
