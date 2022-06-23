# Copyright (c) 2022 Universidad Politécnica de Madrid
# All Rights Reserved
#
# Licensed under the BSD-3-Clause (the "License");
# you may not use this file except in compliance with the License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


from ..behaviour_actions.action_handler import ActionHandler
from rclpy.action import ActionClient
from as2_msgs.action import GoToWaypoint
from geometry_msgs.msg import PoseStamped, Pose
from geographic_msgs.msg import GeoPoseStamped, GeoPose
from as2_msgs.srv import GeopathToPath


class SendGoToWaypoint(ActionHandler):
    def __init__(self, drone, pose, speed, ignore_pose_yaw):
        self._action_client = ActionClient(
            drone, GoToWaypoint, f'{drone.get_drone_id()}/GoToWaypointBehaviour')

        self._drone = drone

        goal_msg = GoToWaypoint.Goal()
        goal_msg.target_pose = self.get_pose(pose)
        goal_msg.max_speed = speed
        goal_msg.ignore_pose_yaw = ignore_pose_yaw

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))


    def get_pose(self, pose):
        if isinstance(pose, Pose):
            return pose
        elif isinstance(pose, PoseStamped):
            return pose.pose
        elif isinstance(pose, GeoPose):
            geopose = GeoPoseStamped()
            geopose.pose = pose
            return self.get_pose(geopose)
        elif isinstance(pose, GeoPoseStamped):
            req = GeopathToPath.Request()
            req.geo_path.poses = [pose]
            resp = self._drone.global_to_local_cli_.call(req)
            if not resp.success:
                self._drone.get_logger().warn("Can't follow path since origin is not set")
                raise self.GoalFailed("GPS service not available")

            return resp.path.poses[0].pose
        else:
            raise self.GoalRejected("Goal format invalid")
