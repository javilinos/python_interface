import rclpy, threading
from rclpy import publisher, spin_once
from rclpy import time
from rclpy import clock
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from time import sleep

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from rclpy.timer import Rate
from geometry_msgs.msg import Quaternion

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


class DroneInterface(Node):
    pose = [0,0,0]
    orientation = [0,0,0]
    odom_lock = threading.Lock()
    fix = [0,0,0]
    gps_lock = threading.Lock()

    def __init__(self,drone_id = "drone0"):
        super().__init__(f'{drone_id}_interface')
        self.namespace = drone_id
        self.odom_sub = self.create_subscription(Odometry, f'{self.namespace}/self_localization/odom', self.odometry_callback, QoSProfile(depth=10))
        self.gps_sub = self.create_subscription(NavSatFix, f'{self.namespace}/platform/gps', self.gps_callback, QoSProfile(depth=10))
        
        spin_thread = threading.Thread(target=self.auto_spin,daemon=True)
        spin_thread.start()

        sleep(0.5)
        print(f'{self.namespace} interface initialized')
    
    def get_drone_id(self):
        return self.namespace

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

    def auto_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            sleep(0.1)


if __name__ == '__main__':
    rclpy.init()
    drone_interface = DroneInterface("drone_sim_8")
    
    while rclpy.ok():
        sleep(0.1)


    ####################
    # rclpy.spin(node)