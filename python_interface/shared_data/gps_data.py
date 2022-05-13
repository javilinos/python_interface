import threading

lock = threading.Lock()

def lock_decor(func):
    def wrapper(self,*args, **kwargs):
        with lock:
            return func(self,*args, **kwargs)
    return wrapper


class GpsData:
    def __init__(self):
        self.fix = [float('nan'), float('nan'), float('nan')]

    @property
    @lock_decor
    def fix(self):
        return self.__fix

    @fix.setter
    @lock_decor
    def fix(self, f):
        self.__fix = f
