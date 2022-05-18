from ..behaviour_actions.action_handler import ActionHandler
from rclpy.action import ActionClient
from as2_msgs.action import Land


class SendLand(ActionHandler):
    def __init__(self, drone, speed=0.0):
        self._action_client = ActionClient(
            drone, Land, f'{drone.get_drone_id()}/LandBehaviour')

        goal_msg = Land.Goal()
        goal_msg.land_speed = speed

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))
