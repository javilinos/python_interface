"""Python interface to easily command drones with AeroStack2.
"""

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

import threading
from time import sleep
from typing import List, Dict, Union

import rclpy
import rclpy.signals
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.parameter import Parameter

from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
from as2_msgs.msg import TrajectoryWaypoints, PlatformInfo, AlertEvent
from as2_msgs.srv import SetOrigin, GeopathToPath, PathToGeopath
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPose
from nav_msgs.msg import Path

from motion_reference_handlers.hover_motion import HoverMotion
from motion_reference_handlers.position_motion import PositionMotion
from motion_reference_handlers.speed_motion import SpeedMotion
from motion_reference_handlers.speed_in_a_plane import SpeedInAPlaneMotion

from python_interface.shared_data.platform_info_data import PlatformInfoData
from python_interface.shared_data.pose_data import PoseData
from python_interface.shared_data.twist_data import TwistData
from python_interface.shared_data.gps_data import GpsData

from python_interface.behaviour_actions.gotowayp_behaviour import SendGoToWaypoint
from python_interface.behaviour_actions.takeoff_behaviour import SendTakeoff
from python_interface.behaviour_actions.followpath_behaviour import SendFollowPath
from python_interface.behaviour_actions.land_behaviour import SendLand

from python_interface.service_clients.arming import Arm, Disarm
from python_interface.service_clients.offboard import Offboard

from python_interface.tools.utils import euler_from_quaternion


STATE = ["DISARMED", "LANDED", "TAKING_OFF", "FLYING", "LANDING", "EMERGENCY"]
YAW_MODE = ["NONE", "YAW_ANGLE", "YAW_SPEED"]
CONTROL_MODE = ["UNSET", "HOVER", "POSITION", "SPEED", "SPEED_IN_A_PLANE",
                "ATTITUDE", "ACRO", "TRAJECTORY", "ACEL"]
REFERENCE_FRAME = ["UNDEFINED_FRAME", "LOCAL_ENU_FRAME",
                   "BODY_FLU_FRAME", "GLOBAL_ENU_FRAME"]


class DroneInterface(Node):
    """Drone abstraction interface. Offers different methods to command the drone.

    :param Node: ROS2 node where the interface will be executed
    :type Node: rclpy.node.Node
    """

    def __init__(self, drone_id: str = "drone0", verbose: bool = False,
                 use_gps: bool = False, use_sim_time: bool = False) -> None:
        """Constructor method

        :param drone_id: drone namespace, defaults to "drone0"
        :type drone_id: str, optional
        :param verbose: output mode, defaults to False
        :type verbose: bool, optional
        :param use_gps: use gps sensor, defaults to False
        :type use_gps: bool, optional
        :param use_sim_time: use simulation time, defaults to False
        :type use_sim_time: bool, optional
        """
        super().__init__(f'{drone_id}_interface', namespace=drone_id)

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        if verbose:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.__info = PlatformInfoData()
        self.pose = PoseData()
        self.twist = TwistData()

        self.namespace = drone_id
        self.get_logger().info(f"Starting {self.drone_id}")

        self.info_sub = self.create_subscription(
            PlatformInfo, 'platform/info', self.__info_callback, qos_profile_system_default)

        # TODO: Synchronious callbacks to pose and twist
        # self.pose_sub = message_filters.Subscriber(self, PoseStamped,
        #   'self_localization/pose', qos_profile_sensor_data.get_c_qos_profile())
        # self.twist_sub = message_filters.Subscriber(self, TwistStamped,
        #   'self_localization/twist', qos_profile_sensor_data.get_c_qos_profile())

        # self._synchronizer = message_filters.ApproximateTimeSynchronizer(
        #     (self.pose_sub, self.twist_sub), 5, 0.01, allow_headerless=True)
        # self._synchronizer.registerCallback(self.pose_callback)

        # State subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.__pose_callback, qos_profile_sensor_data)

        self.twist_sub = self.create_subscription(
            TwistStamped, 'self_localization/twist', self.__twist_callback, qos_profile_sensor_data)

        self.gps_sub = self.create_subscription(
            NavSatFix, 'sensor_measurements/gps', self.__gps_callback, qos_profile_sensor_data)

        translator_namespace = self.namespace
        self.global_to_local_cli_ = self.create_client(
            GeopathToPath, f"{translator_namespace}/geopath_to_path")
        self.local_to_global_cli_ = self.create_client(
            PathToGeopath, f"{translator_namespace}/path_to_geopath")

        self.trajectory_gen_cli = self.create_client(
            SetBool, "traj_gen/run_node")
        if not self.trajectory_gen_cli.wait_for_service(timeout_sec=3):
            self.get_logger().warn("Trajectory generator service not found")
            self.trajectory_gen_cli = None

        self.use_gps = use_gps
        self.gps = GpsData()
        if self.use_gps:
            self.set_origin_cli_ = self.create_client(
                SetOrigin, f"{translator_namespace}/set_origin")
            if not self.set_origin_cli_.wait_for_service(timeout_sec=3):
                self.get_logger().warn("Set Origin not ready")

        self.hover_motion_handler = HoverMotion(self)
        self.position_motion_handler = PositionMotion(self)
        self.speed_motion_handler = SpeedMotion(self)
        self.speed_in_a_plane_motion_handler = SpeedInAPlaneMotion(self)

        self.alert_pub = self.create_publisher(
            AlertEvent, "alert_event", qos_profile_system_default)

        self.keep_running = True
        self.__executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.__auto_spin)
        self.spin_thread.start()

        sleep(0.5)
        self.get_logger().info(f'{self.drone_id} interface initialized')

    def __del__(self) -> None:
        self.__shutdown()

    @property
    def drone_id(self) -> str:
        """Get drone id (namespace).

        :rtype: str
        """
        return self.namespace

    @property
    def info(self) -> Dict[str, Union[bool, str]]:
        """Get drone info.

        :rtype: Dict[str, Union[bool, str]]
        """
        info = self.__get_info()
        return {"connected": bool(info[0]), "armed": bool(info[1]), "offboard": bool(info[2]),
                "state": STATE[info[3]], "yaw_mode": YAW_MODE[info[4]],
                "control_mode": CONTROL_MODE[info[5]], "reference_frame": REFERENCE_FRAME[info[6]]}

    @property
    def position(self) -> List[float]:
        """Get drone position (x, y, z) in m.

        :rtype: List[float]
        """
        return self.pose.position

    @property
    def orientation(self) -> List[float]:
        """Get drone orientation (roll, pitch, yaw) in rad.

        :rtype: List[float]
        """
        return self.pose.orientation

    @property
    def speed(self) -> List[float]:
        """Get drone speed (vx, vy, vz) in m/s.

        :rtype: List[float]
        """
        return self.twist.twist

    @property
    def gps_pose(self) -> List[float]:
        """Get GPS position (lat, lon, alt) in deg and m.

        :rtype: List[float]
        """
        return self.gps.fix

    def __info_callback(self, msg: PlatformInfo) -> None:
        """platform info callback"""
        self.__info.data = [int(msg.connected), int(msg.armed), int(msg.offboard), msg.status.state,
                            msg.current_control_mode.yaw_mode, msg.current_control_mode.control_mode,
                            msg.current_control_mode.reference_frame]

    def __get_info(self) -> List[int]:
        return self.__info.data

    def __pose_callback(self, pose_msg: PoseStamped) -> None:
        """pose stamped callback"""
        self.pose.position = [pose_msg.pose.position.x,
                              pose_msg.pose.position.y,
                              pose_msg.pose.position.z]

        self.pose.orientation = [
            *euler_from_quaternion(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w)]

    def __twist_callback(self, twist_msg: TwistStamped) -> None:
        """twist stamped callback"""
        self.twist.twist = [twist_msg.twist.linear.x,
                            twist_msg.twist.linear.y,
                            twist_msg.twist.linear.z]

    def __gps_callback(self, msg: NavSatFix) -> None:
        """navdata (gps) callback"""
        self.gps.fix = [msg.latitude, msg.longitude, msg.altitude]

    def __set_home(self, gps_pose_: List[float]) -> None:
        """Set home origin"""
        if not self.set_origin_cli_.wait_for_service(timeout_sec=3):
            self.get_logger().error("GPS service not available")
            return

        req = SetOrigin.Request()
        req.origin.latitude = float(gps_pose_[0])
        req.origin.longitude = float(gps_pose_[1])
        req.origin.altitude = float(gps_pose_[2])
        resp = self.set_origin_cli_.call(req)
        if not resp.success:
            self.get_logger().warn("Origin already set")

    def arm(self) -> None:
        """Arm drone.
        """
        sleep(0.1)
        Arm(self)

    def disarm(self) -> None:
        """Disarm drone.
        """
        Disarm(self)

    def offboard(self) -> None:
        """Enable offboard mode.
        """
        Offboard(self)

    def takeoff(self, height: float, speed: float) -> None:
        """Takeoff to given height (m) and given speed (m/s).

        :type height: float
        :type speed: float
        """
        if self.use_gps:
            self.__set_home(self.gps_pose)

        SendTakeoff(self, float(height), float(speed))

    def land(self, speed: float) -> None:
        """Land with given speed (m/s).

        :type speed: float
        """
        SendLand(self, float(speed))

    def __go_to(self, _x: float, _y: float, _z: float,
                speed: float, ignore_yaw: bool, is_gps: bool) -> None:
        if is_gps:
            msg = GeoPose()
            msg.position.latitude = (float)(_x)
            msg.position.longitude = (float)(_y)
            msg.position.altitude = (float)(_z)
        else:
            msg = Pose()
            msg.position.x = (float)(_x)
            msg.position.y = (float)(_y)
            msg.position.z = (float)(_z)
        SendGoToWaypoint(self, msg, speed, ignore_yaw)

    # TODO: ignore_yaw
    def go_to(self, _x: float, _y: float, _z: float, speed: float, ignore_yaw: bool = True) -> None:
        """Go to point (m) with speed (m/s).

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__go_to(_x, _y, _z, speed, ignore_yaw, is_gps=False)

    # TODO: ignore_yaw
    def go_to_point(self, point: List[float],
                    speed: float, ignore_yaw: bool = True) -> None:
        """Go to point (m) with speed (m/s).

        :type point: List[float]
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, ignore_yaw, is_gps=False)

    # TODO: ignore_yaw
    def go_to_gps(self, lat: float, lon: float, alt: float,
                  speed: float, ignore_yaw: bool = True) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__go_to(lat, lon, alt, speed, ignore_yaw, is_gps=True)

    # TODO: ignore_yaw
    def go_to_gps_point(self, waypoint: List[float],
                        speed: float, ignore_yaw: bool = True) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type waypoint: List[float]
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, ignore_yaw, is_gps=True)

    def __follow_path(self, path: Path, speed: float, yaw_mode: int, is_gps: bool = False) -> None:
        path_data = SendFollowPath.FollowPathData(
            path, speed, yaw_mode, is_gps)
        SendFollowPath(self, path_data)

    def follow_path(self, path: Path, speed: float,
                    yaw_mode: int = TrajectoryWaypoints.KEEP_YAW) -> None:
        """Follow path with speed (m/s) and yaw_mode.

        :type path: Path
        :type speed: float
        :param yaw_mode: yaw_mode, defaults to TrajectoryWaypoints.KEEP_YAW
        :type yaw_mode: int, optional
        """
        self.__follow_path(path, speed, yaw_mode)

    def follow_gps_path(self, wp_path: Path, speed: float,
                        yaw_mode: int = TrajectoryWaypoints.KEEP_YAW) -> None:
        """Follow GPS path with speed (m/s) and yaw_mode.

        :type path: Path
        :type speed: float
        :param yaw_mode: yaw_mode, defaults to TrajectoryWaypoints.KEEP_YAW
        :type yaw_mode: int, optional
        """
        self.__follow_path(wp_path, speed, yaw_mode, is_gps=True)

    def emergency_stop(self):
        """Kill switch. BE CAREFUL, motors will stop!
        """
        msg = Bool()
        msg.data = True
        while True:
            self.emergency_stop_pub.publish(msg)
            sleep(0.01)

    def hover(self) -> None:
        """Stop and hover current position.
        """
        if self.trajectory_gen_cli is not None:
            self.get_logger().info("Calling trajectory generator")
            req = SetBool.Request()
            req.data = False
            resp = self.trajectory_gen_cli.call(req)
            if not resp.success:
                self.get_logger().warn("Cannot stop trajectory generator")
        self.hover_motion_handler.send_hover()
        self.get_logger().info("Hover sent")

    # TODO: replace with executor callbacks
    def __auto_spin(self) -> None:
        """Drone inner spin"""
        while rclpy.ok() and self.keep_running:
            self.__executor.spin_once()
            sleep(0.05)

    def __shutdown(self) -> None:
        """Shutdown properly"""
        self.keep_running = False
        self.destroy_subscription(self.info_sub)
        self.destroy_subscription(self.pose_sub)
        self.destroy_subscription(self.gps_sub)

        if self.use_gps:
            self.destroy_client(self.set_origin_cli_)
        self.destroy_client(self.global_to_local_cli_)
        self.destroy_client(self.local_to_global_cli_)

        self.spin_thread.join()
        print("Clean exit")
