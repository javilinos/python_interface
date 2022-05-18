from time import sleep
from service_clients.service_handler import ServiceHandler
from std_srvs.srv import SetBool


class Arm(ServiceHandler):
    def __init__(self, drone, value=True):
        try:
            self._service_client = drone.create_client(
                SetBool, f'{drone.get_drone_id()}/set_arming_state')

        except:
            raise Exception(
                f'{drone.get_drone_id()}/set_arming_state not available')
        request = SetBool.Request()
        request.data = value
        sleep(0.5)
        try:
            super().__init__(self._service_client, request, drone.get_logger())
        except self.ServiceNotAvailable as err:
            drone.get_logger().error(str(err))


class Disarm(Arm):
    def __init__(self, drone):
        super().__init__(drone, False)
