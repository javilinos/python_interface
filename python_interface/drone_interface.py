from matplotlib.pyplot import rc
import rclpy, threading
from rclpy import publisher, spin_once
from rclpy import time
from rclpy import clock
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from time import sleep

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from rclpy.timer import Rate
from geometry_msgs.msg import Quaternion
from as2_msgs.action import FollowPath
from as2_msgs.msg import TrajectoryWaypoints, PlatformInfo
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import math


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians


class SendFollowPath:
    def __init__(self, drone, path):
        self._parent = drone

        self._action_client = ActionClient(self._parent, FollowPath, f'{self._parent.namespace}/FollowPathBehaviour')
        # self._action_client = self._parent.action_client

        goal_msg = FollowPath.Goal()
        goal_msg.trajectory_waypoints = path

        self._action_client.wait_for_server()  # Waiting to action to be available

        # Sending goal
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Waiting to sending goal result
        while not send_goal_future.done():
            sleep(0.1)

        # Check if goal is accepted
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self._parent.get_logger().info('Goal rejected!')
            return
        self._parent.get_logger().info('Goal accepted :)')

        # Getting result
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            sleep(0.1)

        # Check action result
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._parent.get_logger().info("Result: {0}".format(get_result_future.result().result.follow_path_success))
        else:
            self._parent.get_logger().warn("Goal failed with status code: {0}".format(status))

        self._action_client.destroy()

    def feedback_callback(self,feedback_msg):
        self._parent.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.actual_speed))


class DroneInterface(Node):
    info = [0, 0, 0, 0, 0]
    info_lock = threading.Lock()
    pose = [0,0,0]
    orientation = [0,0,0]
    odom_lock = threading.Lock()
    fix = [0,0,0]
    gps_lock = threading.Lock()

    def __init__(self,drone_id = "drone0"):
        super().__init__(f'{drone_id}_interface')
        self.namespace = drone_id
        self.info_sub = self.create_subscription(PlatformInfo, f'{self.get_drone_id()}/platform/info', self.info_callback, QoSProfile(depth=10))
        self.odom_sub = self.create_subscription(Odometry, f'{self.get_drone_id()}/self_localization/odom', self.odometry_callback, QoSProfile(depth=10))
        self.gps_sub = self.create_subscription(NavSatFix, f'{self.get_drone_id()}/platform/gps', self.gps_callback, QoSProfile(depth=10))

        self.globals = []
        self.locals = []
        translator_namespace = ""
        self.fix_pub_ = self.create_publisher(NavSatFix, f"{translator_namespace}/global_pose/fix", 10)
        self.global_sub_ = self.create_subscription(PoseStamped, f"{translator_namespace}/global_pose/ecef", self.global_callback, 10)
        self.local_sub_ = self.create_subscription(PoseStamped, f"{translator_namespace}/local_pose", self.local_callback, 10)
        
        spin_thread = threading.Thread(target=self.auto_spin,daemon=True)
        spin_thread.start()

        sleep(0.5)
        print(f'{self.get_drone_id()} interface initialized')
    
    def get_drone_id(self):
        return self.namespace

    def info_lock_decor(func):
        def wrapper(self,*args, **kwargs):
            with self.info_lock:
                return func(self,*args, **kwargs)
        return wrapper
    
    @info_lock_decor
    def info_callback(self, msg):
        self.info = [int(msg.connected), int(msg.armed), int(msg.offboard), msg.status.state, 
                     msg.current_control_mode.yaw_mode, msg.current_control_mode.control_mode, 
                     msg.current_control_mode.reference_frame]
        
    @info_lock_decor
    def get_info(self):
        return self.info.copy()

    def odom_lock_decor(func):
        def wrapper(self,*args, **kwargs):
            with self.odom_lock:
                return func(self,*args, **kwargs)
        return wrapper
    
    @odom_lock_decor
    def odometry_callback(self, msg):
        self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.orientation = [*euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)]
        
    @odom_lock_decor
    def get_position(self):
        # print(self.pose)
        return self.pose.copy()

    @odom_lock_decor
    def get_orientation(self):
        return self.orientation.copy()

    def gps_lock_decor(func):
        def wrapper(self, *args, **kwargs):
            with self.gps_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @gps_lock_decor
    def gps_callback(self, msg):
        self.fix = [msg.latitude/1e7, msg.longitude/1e7, msg.altitude/1e3]

    @gps_lock_decor
    def get_gps_pose(self):
        return self.fix.copy()

    def global_callback(self, msg):
        self.globals.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def local_callback(self, msg):
        self.locals.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def __follow_path(self, point_list, speed, yaw_mode = TrajectoryWaypoints.KEEP_YAW):
        msg = TrajectoryWaypoints()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
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

        SendFollowPath(self, msg)

    def takeoff(self, height=1.0, speed=0.5):
        self.__follow_path([[0, 0, height]], speed, TrajectoryWaypoints.KEEP_YAW)

    def follow_path(self, path, speed=1.0):
        self.__follow_path(path, speed, TrajectoryWaypoints.PATH_FACING)

    def follow_gps_path(self, wp_path, speed=1.0):
        for wp in wp_path:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "wgs84"
            msg.latitude = float(wp[0])
            msg.longitude = float(wp[1])
            msg.altitude = float(wp[2])
            self.fix_pub_.publish(msg)
        
        sleep(1)
        while len(wp_path) != len(self.locals):
            sleep(0.5)
            # self.get_logger().error("Invalid path.")
            print(self.locals)
            # return

        self.__follow_path(self.locals, speed, TrajectoryWaypoints.PATH_FACING)
        self.locals = []
        self.globals = []

    def land(self):
        self.__follow_path([[0, 0, -0.5]], 0.3, TrajectoryWaypoints.KEEP_YAW)

    def auto_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            sleep(0.1)


if __name__ == '__main__':
    rclpy.init()
    drone_interface = DroneInterface("drone_sim_8")
    
    drone_interface.takeoff()
    print("Takeoff completed\n")
    sleep(1)

    drone_interface.follow_gps_path([[28.14376, -16.5022, 1],
                                 [28.1437, -16.5022, 1],
                                 [28.1437, -16.50235, 1],
                                 [28.14376, -16.50235, 1]])
    print("Path finished")

    drone_interface.land()
    print("Bye!")

    # while rclpy.ok():
    #     sleep(0.1)


    ####################
    # rclpy.spin(node)