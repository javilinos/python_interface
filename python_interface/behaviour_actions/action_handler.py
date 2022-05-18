from time import sleep
from action_msgs.msg import GoalStatus


class ActionHandler:
    TIMEOUT = 3  # seconds

    class ActionNotAvailable(Exception):
        pass

    class GoalRejected(Exception):
        pass

    class GoalFailed(Exception):
        pass

    def __init__(self, action_client, goal_msg, logger):
        self._logger = logger

        # Wait for Action availability
        if not action_client.wait_for_server(timeout_sec=self.TIMEOUT):
            raise self.ActionNotAvailable('Action Not Available')

        # Sending goal
        send_goal_future = action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Waiting to sending goal result
        while not send_goal_future.done():
            sleep(0.1)

        # Check if goal is accepted
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise self.GoalRejected('Goal Rejected')
        self._logger.info('Goal accepted :)')

        # Getting result
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            sleep(0.1)

        # Check action result
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info("Result: {0}".format(get_result_future.result().result))
        else:
            raise self.GoalFailed("Goal failed with status code: {0}".format(status))

        action_client.destroy()

    def feedback_callback(self, feedback_msg):
        self._logger.info('Received feedback: {0}'.format(feedback_msg.feedback))
