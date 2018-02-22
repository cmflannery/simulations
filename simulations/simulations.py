""" simulations.py
Simulations is a python based flight simulation package
for rocket and missle trajectory analysis """
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import units
import atmos


class RocSim(object):
    def __init__(self, state, timestep, duration, Isp, pexit):
        """ RocSim must be initialized with a state variable.
        
        state = [
            time,
            velocity,
            path_angle,
            heading_angle,
            latitude,
            longitude,
            altitude,
            pa,
            mass,
            heat,
            thrust,
            thrust_angle,
            lift_coefficient,
            drag_coefficient,
            bank_angle
            ]

        The state variable always represents the latest state.

        engine = [
            Isp,
            pexit,
            Ae,
            thrust_sl
        ]
        """
        self.state0 = [[i] for i in state]
        keys = ['time','velocity','path_angle','heading_angle','latitude','longitude',\
                'altitude','mass','heat','thrust_angle','lift_coefficient','drag_coefficient',\
                'bank_angle']
        self.stateDict = dict(zip(keys,self.state0))
        self.start = self.state0[0][0]
        self.timestep = timestep  # timestep is the size of the time between states in seconds
        self.duration = duration  # duration is the total time in seconds of the run
        self.pexit = pexit
        self.constants()

    def run_simulation(self):
        for i in range(self.start, self.start+self.duration):
            self.update_state()
        plt.plot(self.stateDict['velocity'])
        plt.show()

    def constants(self):
        self.g0 = 9.81  # [m/s^2]
        self.Re = 6378000  # [m]

    def update_state(self):
        self.stateDict['velocity'].append(self.update_velocity)
        self.stateDict['pa'].append(self.update_pa)

    def update_pa(self,i):
        """ return an updated value for pressure """
        altitude = self.stateDict['altitude'][i]
        return atmos.SimpleAtmos(altitude,'m')

    def update_thrust(self,i):
        Aexit = self.engine['Aexit']
        pexit = self.engine['pexit']
        pa = self.stateDict['pa'][i]
        return self.engine['thrust_sl'] + (pexit-pa)*Aexit
    
    def update_velocity(self):
        thrust = self.stateDict['thrust']
        thrust_angle = self.stateDict['thrust_angle']
        mass = self.stateDict['mass']
        radius = self.stateDict['radius']
        flight_path_angle = self.stateDict['flight_path_angle']  # degrees
        drag = 0
        return (thrust*np.cos(thrust_angle)-drag)/mass - self.g0*(Re/R)**2*sin(flight_path_angle)

    def update_mass(self):
        thrust = self.state['thrust']
        g0 = self.g0
        Isp = self.Isp


def test():
    state = [
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        100,
        0,
        1000,
        0,
        0,
        0,
        0
    ]
    itsatest = RocSim(state,0.01,10)
    itsatest.run_simulation()


if __name__ == '__main__':
    test()