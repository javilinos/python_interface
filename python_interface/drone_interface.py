import rclpy, threading
from rclpy import publisher, spin_once
from rclpy import time
from rclpy import clock
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from time import sleep

from nav_msgs.msg import Odometry
from rclpy.timer import Rate


class DroneInterface(Node):
    pose = [0,0,0]
    odom_lock = threading.Lock()

    def __init__(self,drone_id = "drone0"):
        super().__init__(f'{drone_id}_interface')
        self.namespace = drone_id
        self.odom_sub = self.create_subscription(Odometry, f'{self.namespace}/self_localization/odom', self.odometry_callback, QoSProfile(depth=10))
        
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
        

    @odom_lock_decor
    def get_position(self):
        # print(self.pose)
        return self.pose.copy()

    def auto_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            sleep(0.1)


if __name__ == '__main__':
    rclpy.init()
    drone_interface = DroneInterface("drone_sim_11")
    
    while rclpy.ok():
        sleep(0.1)


    ####################
    # rclpy.spin(node)