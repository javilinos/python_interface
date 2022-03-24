import threading

lock = threading.Lock()

def lock_decor(func):
    def wrapper(self,*args, **kwargs):
        with lock:
            return func(self,*args, **kwargs)
    return wrapper


class PoseData:
    def __init__(self):
        self.pose = [float('nan'), float('nan'), float('nan')]

    @property
    @lock_decor
    def pose(self):
        return self.__pose

    @pose.setter
    @lock_decor
    def pose(self, p):
        self.__pose = p
