""" simulations.py
Simulations is a python based flight simulation package
for rocket and missle trajectory analysis """
import numpy as np


class rocket():
    def __init__(self):
        self.constants()

    def constants(self):
        self.g0 = 9.81
        return 0
    
    @property
    def mp(self):
        return self.__mp

    @mp.setter
    def mp(self, value):
        self.__mp = value

    @property
    def mf(self):
        return self.__mf

    @mf.setter
    def mf(self, value):
        self.__mf = value

    @property
    def mpay(self):
        return self.__mpay

    @mpay.setter
    def mpay(self, value):
        self.__mpay = value
    
    @property
    def mlaunch(self):
        """ total launch mass """
        return self.mp + self.mf + self.mpay

    @property
    def MR(self):
        return self.mlaunch/self.mf

    @property
    def tburn(self):
        return self.__tburn

    @tburn.setter
    def tburn(self, value):
        self.__tburn = value

    @property
    def thrust(self):
        return self.__thrust

    @thrust.setter
    def thrust(self, value):
        self.__thrust = value
    
    @property
    def launch_weight(self):
        return self.mlaunch*self.g0

    @property
    def launch_angle(self):
        return self.__launch_angle

    @launch_angle.setter
    def launch_angle(self, value):
        self.__launch_angle = value

    @property
    def a0y(self):
        return self.g0*(np.sin(np.deg2rad(self.launch_angle))*self.thrust/self.launch_weight-1)

    @property
    def a0x(self):
        return self.g0*np.cos(np.deg2rad(self.launch_angle))*self.thrust/self.launch_weight

    @property
    def a0(self):
        return (self.a0x**2 + self. a0y**2)**0.5

    @property
    def upy(self):
        """ vertical velocity at end of powered flight """
        return 1


def test():
    thing = rocket()
    thing.mp = 300  # propellant mass
    thing.mf = 100  # dry vehicle mass
    thing.mpay = 50  # payload mass
    thing.tburn = 100  # burn time
    thing.thrust = 5000  # thrust
    thing.launch_angle = 80

    print('weight =', thing.launch_weight)
    print('a0 = ', thing.a0)
    print('a0y = ', thing.a0y)
    print('a0x = ', thing.a0x)


if __name__ == '__main__':
    test()