from .action_executor import *
from .morse_executor import *
from .ros_executor import *
from .morse_ros_executor import *
from .hyper_executor import *

def all_subclasses(classname):
    direct_subclasses = eval(classname).__subclasses__()
    return direct_subclasses + [g for s in direct_subclasses
                                    for g in all_subclasses(s.__name__)]

""" Create an Executor from its name """
def create_executor(name, **kwargs):
    for cls in all_subclasses('AbstractActionExecutor'):
        if cls._name == name:
            return cls(**kwargs)
    return None

""" Get all available executors """
def executors():
    return [cls._name for cls in all_subclasses('AbstractActionExecutor')]
