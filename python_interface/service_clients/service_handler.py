from time import sleep


class ServiceHandler:
    TIMEOUT = 3  # seconds

    class ServiceNotAvailable(Exception):
        pass

    class ServiceFailed(Exception):
        pass

    def __init__(self, service_client, request_msg, logger):
        self._logger = logger
        response = None

        # Wait for Action availability
        if not service_client.wait_for_service(timeout_sec=self.TIMEOUT):
            raise self.ServiceNotAvailable('Service not Available')

        # Sending goal
        response = service_client.call(request_msg)

        return response
