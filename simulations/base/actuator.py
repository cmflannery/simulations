from abc import ABC, abstractmethod


class Actuator(ABC):

    @abstractmethod
    def command(self):
        pass
