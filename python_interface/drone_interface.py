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

"""
A collection of utils to easily command drones with AeroStack2.
"""

import threading
from time import sleep

import rclpy
import rclpy.signals
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from as2_msgs.msg import TrajectoryWaypoints, PlatformInfo
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose
from as2_msgs.srv import SetOrigin, GeopathToPath, PathToGeopath

from .shared_data.platform_info_data import PlatformInfoData
from .shared_data.odom_data import OdomData
from .shared_data.gps_data import GpsData

from .behaviour_actions.gotowayp_behaviour import SendGoToWaypoint
from .behaviour_actions.takeoff_behaviour import SendTakeoff
from .behaviour_actions.followpath_behaviour import SendFollowPath
from .behaviour_actions.land_behaviour import SendLand

from .service_clients.arming import Arm, Disarm
from .service_clients.offboard import Offboard

from .tools.utils import euler_from_quaternion


STATE = ["DISARMED", "LANDED", "TAKING_OFF", "FLYING", "LANDING", "EMERGENCY"]
YAW_MODE = ["YAW_ANGLE", "YAW_SPEED"]
CONTROL_MODE = ["POSITION_MODE", "SPEED_MODE", "SPEED_IN_A_PLANE",
                "ACCEL_MODE", "ATTITUDE_MODE", "ACRO_MODE", "UNSET"]
REFERENCE_FRAME = ["LOCAL_ENU_FRAME", "BODY_FLU_FRAME", "GLOBAL_ENU_FRAME"]


class DroneInterface(Node):
    def __init__(self, drone_id="drone0", verbose=False):
        super().__init__(f'{drone_id}_interface')

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        if not verbose:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

        self.info = PlatformInfoData()
        self.odom = OdomData()
        self.gps = GpsData()

        self.namespace = drone_id
        self.info_sub = self.create_subscription(
            PlatformInfo, f'{self.get_drone_id()}/platform/info', self.info_callback, qos_profile_system_default)
        self.odom_sub = self.create_subscription(
            Odometry, f'{self.get_drone_id()}/self_localization/odom', self.odometry_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(
            NavSatFix, f'{self.get_drone_id()}/sensor_measurements/gps', self.gps_callback, qos_profile_sensor_data)

        translator_namespace = ""
        self.global_to_local_cli_ = self.create_client(
            GeopathToPath, f"{translator_namespace}/geopath_to_path")
        self.local_to_global_cli_ = self.create_client(
            PathToGeopath, f"{translator_namespace}/path_to_geopath")

        self.set_origin_cli_ = self.create_client(
            SetOrigin, f"{translator_namespace}/set_origin")
        if not self.set_origin_cli_.wait_for_service(timeout_sec=3):
            self.get_logger().error("Set Origin not ready")

        self.keep_running = True
        self.__executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.auto_spin)
        self.spin_thread.start()

        sleep(0.5)
        print(f'{self.get_drone_id()} interface initialized')

    def __del__(self):
        self.shutdown()

    def get_drone_id(self):
        return self.namespace

    def info_callback(self, msg):
        self.info.data = [int(msg.connected), int(msg.armed), int(msg.offboard), msg.status.state,
                          msg.current_control_mode.yaw_mode, msg.current_control_mode.control_mode,
                          msg.current_control_mode.reference_frame]

    def __get_info(self):
        return self.info.data

    def get_info(self):
        info = self.__get_info()
        return {"connected": bool(info[0]), "armed": bool(info[1]), "offboard": bool(info[2]), "state": STATE[info[3]],
                "yaw_mode": YAW_MODE[info[4]], "control_mode": CONTROL_MODE[info[5]], "reference_frame": REFERENCE_FRAME[info[6]]}

    def odometry_callback(self, msg):
        self.odom.pose = [msg.pose.pose.position.x,
                          msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom.orientation = [*euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)]

    def get_position(self):
        return self.odom.pose

    def get_orientation(self):
        return self.odom.orientation

    def gps_callback(self, msg):
        self.gps.fix = [msg.latitude, msg.longitude, msg.altitude]

    def get_gps_pose(self):
        return self.gps.fix

    def __set_home(self):
        if not self.set_origin_cli_.wait_for_service(timeout_sec=3):
            self.get_logger().error("GPS service not available")
            return

        gps_pose = self.get_gps_pose()

        req = SetOrigin.Request()
        req.origin.latitude = float(gps_pose[0])
        req.origin.longitude = float(gps_pose[1])
        req.origin.altitude = float(gps_pose[2])
        resp = self.set_origin_cli_.call(req)
        if not resp.success:
            self.get_logger().warn("Origin already set")

    def __follow_path(self, path, speed, yaw_mode, is_gps=False):
        path_data = SendFollowPath.FollowPathData(
            path, speed, yaw_mode, is_gps)
        SendFollowPath(self, path_data)

    def takeoff(self, height=1.0, speed=0.5):
        self.__set_home()

        # self.__follow_path([self.get_position()[:2] + [height]], speed, TrajectoryWaypoints.PATH_FACING)

        SendTakeoff(self, float(height), float(speed))

    def follow_path(self, path, speed=1.0, yaw_mode=TrajectoryWaypoints.KEEP_YAW):
        self.__follow_path(path, speed, yaw_mode)

    def follow_gps_path(self, wp_path, speed=1.0, yaw_mode=TrajectoryWaypoints.KEEP_YAW):
        self.__follow_path(wp_path, speed, yaw_mode, is_gps=True)

    def arm(self):
        sleep(0.1)
        Arm(self)

    def disarm(self):
        Disarm(self)

    def offboard(self):
        Offboard(self)

    # TEMPORAL
    def follow_gps_wp(self, wp_list, speed=1.0):
        for pnt in wp_list:
            self.__follow_path(
                [pnt], speed, TrajectoryWaypoints.PATH_FACING, is_gps=True)

    def land(self, speed=0.5):
        SendLand(self, float(speed))
        # pose  = self.get_position()[:2]
        # self.__follow_path([pose + [-0.5]], 0.3, TrajectoryWaypoints.KEEP_YAW)

    def __go_to(self, x, y, z, speed, ignore_yaw, is_gps):
        if is_gps:
            msg = GeoPose()
            msg.position.latitude = (float)(x)
            msg.position.longitude = (float)(y) 
            msg.position.altitude = (float)(z)
        else:
            msg = Pose()
            msg.position.x = (float)(x)
            msg.position.y = (float)(y)
            msg.position.z = (float)(z)
        SendGoToWaypoint(self, msg, speed, ignore_yaw)

    def go_to(self, x, y, z, speed, ignore_yaw=True):
        self.__go_to(x, y, z, speed, ignore_yaw, is_gps=False)

    # TODO: python overloads?
    # def go_to(self, point, speed, ignore_yaw=True):
    #     self.__go_to(point[0], point[1], point[2], speed, ignore_yaw, is_gps=False)

    def go_to_gps(self, lat, lon, alt, speed, ignore_yaw=True):
        self.__go_to(lat, lon, alt, speed, ignore_yaw, is_gps=True)

    # TODO: python overloads?
    # def go_to_gps(self, waypoint, speed, ignore_yaw=True):
    #     self.__go_to(waypoint[0], waypoint[1], waypoint[2], speed, ignore_yaw, is_gps=True)

    def auto_spin(self):
        while rclpy.ok() and self.keep_running:
            self.__executor.spin_once()
            sleep(0.05)

    def shutdown(self):
        self.keep_running = False
        self.destroy_subscription(self.info_sub)
        self.destroy_subscription(self.odom_sub)
        self.destroy_subscription(self.gps_sub)

        # self.destroy_client(self.set_origin_cli_)
        self.destroy_client(self.global_to_local_cli_)
        self.destroy_client(self.local_to_global_cli_)

        self.spin_thread.join()
        print("Clean exit")

