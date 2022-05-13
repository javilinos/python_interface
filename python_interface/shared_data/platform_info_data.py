import threading

lock = threading.Lock()

def lock_decor(func):
    def wrapper(self,*args, **kwargs):
        with lock:
            return func(self,*args, **kwargs)
    return wrapper


class PlatformInfoData:
    def __init__(self):
        self.data = [0, 0, 0, 0, 0]

    @property
    @lock_decor
    def data(self):
        return self.__data

    @data.setter
    @lock_decor
    def data(self, d):
        self.__data = d
