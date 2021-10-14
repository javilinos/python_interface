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
from rclpy.action import ActionClient
from rclpy.node import Node

from aerostack2_msgs.action import FollowPath
from aerostack2_msgs.msg import TrajectoryWaypoints
from geometry_msgs.msg import PoseStamped

class SendFollowPath(Node):

    def __init__(self,point_list,speed,yaw_mode = TrajectoryWaypoints.KEEP_YAW, drone_id='drone0'):
        rclpy.init(args=None)
        super().__init__('send_follow_path_action_client')
        
        action_name = 'FollowPathBehaviour'
        
        if self.get_namespace() == '/':
            action_name = drone_id + '/' + action_name

        self._action_client = ActionClient(self, FollowPath, action_name)
        self.sendPath(point_list,speed,yaw_mode)
        
    def sendPath(self,point_list,speed,yaw_mode = TrajectoryWaypoints.KEEP_YAW):

        msg = TrajectoryWaypoints()
        msg.header.stamp = self.get_clock().now().to_msg()
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

        goal_msg = FollowPath.Goal()

        goal_msg.trajectory_waypoints = msg

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedbackCallback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        rclpy.spin(self)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.follow_path_success))
        rclpy.shutdown()

    def feedbackCallback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.actual_speed))


def main(args=None):

    point_list = [[0,0,1]]
    # point_list = [[1,2,1],[-1,-1,1],[0,0,1]]

    SendFollowPath(point_list,0.5,TrajectoryWaypoints.KEEP_YAW)

    print("Take off completed successfully")

    # # point_list = [[0,0,5]]
    dist = 1.5
    height = 1.5
    point_list = [[dist,dist,height],[-dist,dist,height],[-dist,-dist,height],[dist,-dist,height],[0,0,height]]
    SendFollowPath(point_list,2,TrajectoryWaypoints.KEEP_YAW)
    SendFollowPath(point_list,1.5,TrajectoryWaypoints.PATH_FACING)

    print("Path completed successfully")

    point_list = [[0,0,-1]]
    SendFollowPath(point_list,0.3,TrajectoryWaypoints.KEEP_YAW)

    print("Landing completed successfully")


if __name__ == '__main__':
    main()