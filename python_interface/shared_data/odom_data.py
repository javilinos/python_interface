from shared_data.pose_data import PoseData
from shared_data.orientation_data import OrientationData


class OdomData:
    def __init__(self):
        self.__pose = PoseData()
        self.__orientation = OrientationData()

    @property
    def pose(self):
        return self.__pose.pose

    @pose.setter
    def pose(self, p):
        self.__pose.pose = p

    @property
    def orientation(self):
        return self.__orientation.orientation

    @orientation.setter
    def orientation(self, o):
        self.__orientation.orientation = o