from behaviour_actions.action_handler import ActionHandler
from rclpy.action import ActionClient
from as2_msgs.action import TakeOff


class SendTakeoff(ActionHandler):
    def __init__(self, drone, height, speed):
        self._action_client = ActionClient(drone, TakeOff, f'{drone.get_drone_id()}/TakeOffBehaviour')

        goal_msg = TakeOff.Goal()
        goal_msg.takeoff_height = height
        goal_msg.takeoff_speed = speed

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))