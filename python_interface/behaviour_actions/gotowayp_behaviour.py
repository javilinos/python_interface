from behaviour_actions.action_handler import ActionHandler
from rclpy.action import ActionClient
from as2_msgs.action import GoToWaypoint

class SendGoToWaypoint(ActionHandler):
    def __init__(self, drone, pose, speed, ignore_pose_yaw):
        self._action_client = ActionClient(drone, GoToWaypoint, f'{drone.get_drone_id()}/GoToWaypointBehaviour')

        goal_msg = GoToWaypoint.Goal()
        goal_msg.target_pose = pose
        goal_msg.max_speed = speed
        goal_msg.ignore_pose_yaw = ignore_pose_yaw

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))
