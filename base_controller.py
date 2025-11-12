from abc import ABC, abstractmethod    #ABC=Bastract base class

class BaseController(ABC):
    "blueprint that all contrller will follow"
    "every controller must implement these methods and reset"

    def __init__(self,name):
        self.name=name
        self.control_output=None

        @abstractmethod
        def compute_control(self,t,state,reference):
            #all compute control output given current state and reference
            pass

        @abstractmethod
        def reset(self):
                pass            # reset the controller to initial state

        def get_control_output(self):
            return self.control_output   #to get the latest control output

        def get_control_info():
            return {'name':self.name,
                    'type':self.__class__.__name}



