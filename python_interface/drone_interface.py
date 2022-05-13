import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from time import sleep

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from as2_msgs.msg import TrajectoryWaypoints, PlatformInfo
from geometry_msgs.msg import Pose
from as2_msgs.srv import SetOrigin, GeopathToPath, PathToGeopath

from shared_data.platform_info_data import PlatformInfoData
from shared_data.odom_data import OdomData
from shared_data.gps_data import GpsData

from behaviour_actions.gotowayp_behaviour import SendGoToWaypoint
from behaviour_actions.takeoff_behaviour import SendTakeoff
from behaviour_actions.followpath_behaviour import SendFollowPath
from behaviour_actions.land_behaviour import SendLand

from tools.utils import euler_from_quaternion


STATE = ["DISARMED", "LANDED", "TAKING_OFF", "FLYING", "LANDING", "EMERGENCY"]
YAW_MODE = ["YAW_ANGLE", "YAW_SPEED"]
CONTROL_MODE = ["POSITION_MODE", "SPEED_MODE", "SPEED_IN_A_PLANE", "ACCEL_MODE", "ATTITUDE_MODE", "ACRO_MODE", "UNSET"]
REFERENCE_FRAME = ["LOCAL_ENU_FRAME", "BODY_FLU_FRAME", "GLOBAL_ENU_FRAME"]


class DroneInterface(Node):
    def __init__(self,drone_id = "drone0", verbose=True):
        super().__init__(f'{drone_id}_interface')

        if not verbose:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

        self.info = PlatformInfoData()
        self.odom = OdomData()
        self.gps = GpsData()

        self.namespace = drone_id
        self.info_sub = self.create_subscription(PlatformInfo, f'{self.get_drone_id()}/platform/info', self.info_callback, qos_profile_system_default)
        self.odom_sub = self.create_subscription(Odometry, f'{self.get_drone_id()}/self_localization/odom', self.odometry_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, f'{self.get_drone_id()}/sensor_measurements/gps', self.gps_callback, qos_profile_sensor_data)

        translator_namespace = ""
        self.global_to_local_cli_ = self.create_client(GeopathToPath, f"{translator_namespace}/geopath_to_path")
        self.local_to_global_cli_ = self.create_client(PathToGeopath, f"{translator_namespace}/path_to_geopath")
        
        self.set_origin_cli_ = self.create_client(SetOrigin, f"{translator_namespace}/set_origin")
        if not self.set_origin_cli_.wait_for_service(timeout_sec=10):
            self.get_logger().error("Set Origin not ready")
        
        self.keep_running = True
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
        self.odom.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom.orientation = [*euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)]
        
    def get_position(self):
        return self.odom.pose

    def get_orientation(self):
        return self.odom.orientation

    def gps_callback(self, msg):
        self.gps.fix = [msg.latitude/1e7, msg.longitude/1e7, msg.altitude/1e3]

    def get_gps_pose(self):
        return self.gps.fix

    def __set_home(self):
        gps_pose = self.get_gps_pose()

        req = SetOrigin.Request()
        req.origin.latitude = float(gps_pose[0])
        req.origin.longitude = float(gps_pose[1])
        req.origin.altitude = float(gps_pose[2])
        resp = self.set_origin_cli_.call(req)
        if not resp.success:
            self.get_logger().warn("Origin already set")

    def __follow_path(self, path, speed=1.0, yaw_mode = TrajectoryWaypoints.KEEP_YAW, is_gps=False):
        path_data = SendFollowPath.FollowPathData(path, speed, yaw_mode, is_gps)
        SendFollowPath(self, path_data)

    def takeoff(self, height=1.0, speed=0.5):
        # self.__set_home()

        # self.__follow_path([self.get_position()[:2] + [height]], speed, TrajectoryWaypoints.PATH_FACING)

        SendTakeoff(self, float(height), float(speed))

    def follow_path(self, path, speed=1.0):
        self.__follow_path(path, speed, TrajectoryWaypoints.PATH_FACING)

    def follow_gps_path(self, wp_path, speed=1.0):
        self.__follow_path(wp_path, speed, TrajectoryWaypoints.PATH_FACING, is_gps=True)

    # TEMPORAL
    def follow_gps_wp(self, wp_list, speed=1.0):
        for pnt in wp_list:
            self.__follow_path([pnt], speed, TrajectoryWaypoints.PATH_FACING, is_gps=True)
        
    def land(self, speed=0.5):
        SendLand(self, float(speed))
        # pose  = self.get_position()[:2]
        # self.__follow_path([pose + [-0.5]], 0.3, TrajectoryWaypoints.KEEP_YAW)

    def __go_to(self, x, y, z, speed, ignore_yaw):
        msg = Pose()
        msg.position.x = (float)(x)
        msg.position.y = (float)(y)
        msg.position.z = (float)(z)
        SendGoToWaypoint(self, msg, speed, ignore_yaw)

    def go_to(self, x, y, z, speed=2.0, ignore_yaw=True):
        self.__go_to(x, y, z, speed, ignore_yaw)

    def auto_spin(self):
        while rclpy.ok() and self.keep_running:
            rclpy.spin_once(self)
            sleep(0.1)
    
    def shutdown(self):
        self.destroy_subscription(self.info_sub)
        self.destroy_subscription(self.odom_sub)
        self.destroy_subscription(self.gps_sub)

        self.destroy_client(self.set_origin_cli_)
        self.destroy_client(self.global_to_local_cli_)
        self.destroy_client(self.local_to_global_cli_)
        
        self.keep_running = False
        self.spin_thread.join()
        rclpy.shutdown()
        print("Clean exit")
        

if __name__ == '__main__':
    rclpy.init()

    drone_interface = DroneInterface("drone_sim_0", verbose=True)

    drone_interface.takeoff(3, 2)
    print("Takeoff completed\n")
    sleep(1)

    drone_interface.go_to(0, 5, 3)
    drone_interface.go_to(5, 0, 2)
    # drone_interface.go_to(5, 5, 0)
    # drone_interface.go_to(0, 5, 3)
    # drone_interface.go_to(0, 0, 3)

    # drone_interface.follow_path([[5, 0, 3],
    #                                 [5, 5, 3],
    #                                 [0, 5, 3],
    #                                 [0, 0, 3]], 5)

    # drone_interface.follow_gps_path([[28.14376, -16.5022, 3],
    #                                 [28.1437, -16.5022, 3],
    #                                 [28.1437, -16.50235, 3],
    #                                 [28.14376, -16.50235, 3]], 5)
    print("Path finished")

    drone_interface.land(0.2)
    drone_interface.shutdown()

    print("Bye!")