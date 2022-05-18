from ..service_clients.service_handler import ServiceHandler
from std_srvs.srv import SetBool
from time import sleep


class Offboard(ServiceHandler):
    def __init__(self, drone, value=True):

        self._service_client = drone.create_client(
            SetBool, f'{drone.get_drone_id()}/set_offboard_mode')

        request = SetBool.Request()
        request.data = value
        sleep(0.5)
        try:
            super().__init__(self._service_client, request, drone.get_logger())
        except self.ServiceNotAvailable as err:
            drone.get_logger().error(str(err))
