import threading

lock = threading.Lock()

def lock_decor(func):
    def wrapper(self,*args, **kwargs):
        with lock:
            return func(self,*args, **kwargs)
    return wrapper


class OrientationData:
    def __init__(self):
        self.orientation = [float('nan'), float('nan'), float('nan')]

    @property
    @lock_decor
    def orientation(self):
        return self.__orientation

    @orientation.setter
    @lock_decor
    def orientation(self, o):
        self.__orientation = o
