""" simulations.py
Simulations is a python based flight simulation package
for rocket and missle trajectory analysis. """
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import units
import atmos


class Rocket(object):
    """ Rocket is a simulation class for rocket simulations.
    
    This module performs calculations for the estimation of
    launch vehicle sizing and trajectory simulation for 
    vertical launch vehicles.
    
    A list of assumptions, capabilities, and limitations
    will be added here as features are solidified. """
    
    def __init__(self, initialConditions, engines, burntime, timestep=1):
        """ Initialization of the Rocket simulation class

        Args:
            initialConditions: expects dictionary containing
            initial conditions for launch vehicle.
             -> Required keywords:
                time,               # [s]
                velocity,           # [m/s]
                flight_angle,       # [rad] vertical flight path angle
                flight_heading,     # [rad] flight path heading
                latitude,           # [rad]
                longitude,          # [rad]
                altitude,           # [m]
                mass,               # [kg]
                heat,               # [J]
                thrust_sl,          # [N] sea-level thrust
                thrust_angle,       # [rad]
                lift_coefficient,   # [1]
                bank_angle          # [rad]
            
            engines:
            -> Required keywords:
                thrust_sl:          # [N]
                Isp:                # [s]
                Ae:                 # [m^2]
                nengines:           # [1]
            
            burntime: length of burn in seconds
        Keyword Args:
            timestep: (optional), timestep in seconds. Default
                timestep is 1s
        Returns:
            0: Completed with no errors
        """
        
        requiredArgs = [
            'time',
            'velocity',
            'flight_angle',
            'flight_heading',
            'latitude',
            'longitude',
            'altitude',
            'mass',
            'heat',
            'lift_coefficient',
            'bank_angle'
            'thrust_sl',
            'thrust_angle',
            'Ae',
            'Isp',
        ]

        for arg in requiredArgs:
            if arg not in (initialConditions or engines):
                pass

        self.initialConditions = initialConditions
        self.engines = engines
        self.burntime = burntime
        self.timestep = timestep
        self.CONST()

    def run(self, stopTime=None, stopApogee=None):
        """ runs simulation
        
        Automatically ends simulation when the vehicle impacts
        the ground, or reaches a stable orbit.
        """

        # initialize arrays with values from initialConditions
        self.time               = [self.initialConditions['time']]
        self.velocity           = [self.initialConditions['velocity']]
        self.flight_angle       = [self.initialConditions['flight_angle']]
        self.flight_heading     = [self.initialConditions['flight_heading']]
        self.latitude           = [self.initialConditions['latitude']]
        self.longitude          = [self.initialConditions['longitude']]
        self.altitude           = [self.initialConditions['altitude']]
        self.mass               = [self.initialConditions['mass']]
        self.heat               = [self.initialConditions['heat']]
        self.lift_coefficient   = [self.initialConditions['lift_coefficient']]
        self.bank_angle         = [self.initialConditions['bank_angle']]
        self.drag               = [self.calc_Cd(0)]
        
        # initialize arrays with values from engines
        self.nengines           = self.engines['nengines']
        self.thrust             = [self.engines['thrust_sl']*self.nengines]
        self.thrust_angle       = [self.engines['thrust_angle']]
        self.Ae                 = [self.engines['Ae']]
        self.Isp                = self.engines['Isp']
        self.mdot               = self.thrust[0]/(self.g0*self.Isp)

        # initialize additional values
        self.acceleration    = [0]
        self.R               = [self.Rearth]  # [m] initial distance to the center of the earth

        self.runIter = 0  # iterator
        while True:
            self.time.append(self.time[self.runIter] + self.timestep)
            self.R.append(self.Rearth + self.altitude[self.runIter]) 
            T, rho, sos = self.STDATM(self.altitude[self.runIter])  # Thermoproperties

            M = self.velocity[self.runIter]/sos
            Cd = self.calc_Cd(M)

            # calculate altitude, velocity, and acceleration
            self.altitude.append(self.altitude[self.runIter] + self.calc_dalt())
            self.velocity.append(self.velocity[self.runIter] + self.calc_deltaV())
            self.acceleration.append(self.calc_accel())

            # Thrust
            if self.time[self.runIter] <= self.burntime:
                self.thrust.append(self.thrust[0])
                self.mass.append(self.mass[self.runIter] - self.mdot*self.timestep)
            else:
                self.thrust.append(0)
                self.mass.append(self.mass[self.runIter])
            self.flight_heading.append(self.flight_heading[0])  # initial values until calcs added
            self.flight_angle.append(self.flight_angle[0])      # initial values until calcs added
            self.thrust_angle.append(self.thrust_angle[0])
            self.drag.append(self.drag[0])

            # END CONDITIONS
            if (self.altitude[self.runIter] < 1000 and self.time[self.runIter] > self.burntime) or self.time[self.runIter] > 10000:
                break

            self.runIter += 1
        return (self.altitude, self.velocity, self.acceleration, self.mass, self.time, self.thrust)

    def calc_Cd(self, M):
        return 0.15 + 0.6*M**2*np.exp(-M**2)

    def calc_drag(self, vel, rho, S, Cd):
        return 1/2*rho*vel**2*S*Cd

    def calc_thrust(self, thrust_sl=None, Ae=None, pe=None, pa=None):
        """ calc_thrust determines the thrust """

        return thrust_sl + (pe-pa)*Ae

    def calc_accel(self, thrust=None, thrust_angle=None,
                   drag=None, mass=None, g0=None, R=None,
                   flight_heading=None, timestep=None):
        """ calc_accel is a method of Rocket

        This method is typically used to update the acceleration values during
        simulation runs.

        Note:
            All arguments are optional. If no arguments are thrown,
            the method will return the calculated delta velocity
            based on the current timestep values.

        Args:
            thrust:         # [N]
            thrust_angle:   # [rad] angle
            drag:           # [N]
            mass:           # [kg]
            flight_heading:  # [rad] angle
            R:              # [m] radius from center of the Earth
            timestep:        # [s] timestep

        Returns:
            accel: acceleration value of the rocket
        """
        
        i = self.runIter
        if not thrust:
            thrust = self.thrust[i]
        if not thrust_angle:
            thrust_angle = self.thrust_angle[i]
        if not drag:
            drag = self.drag[i]
        if not mass:
            mass = self.mass[i]
        if not R:
            R = self.R[i]
        if not flight_heading:
            flight_heading = self.flight_heading[i]
        if not timestep:
            timestep = self.timestep

        return self.dVdt(thrust, thrust_angle, drag, mass, R, flight_heading)

    def calc_deltaV(self, thrust=None, thrust_angle=None,
                    drag=None, mass=None, g0=None, R=None,
                    flight_heading=None, timestep=None):
        """ calc_velocity is a method of Rocket

        This method is typically used to update the velocity values during
        simulation runs.

        Note:
            All arguments are optional. If no arguments are thrown,
            the method will return the calculated delta velocity
            based on the current timestep values.

        Args:
            thrust:         # [N]
            thrust_angle:   # [rad] angle
            drag:           # [N]
            mass:           # [kg]
            flight_heading: # [rad] angle
            R:              # [m] radius from center of the Earth
            timestep:       # [s] timestep

        Returns:
            dV: change in velocity of the rocket
        """

        i = self.runIter
        if not thrust:
            thrust = self.thrust[i]
        if not thrust_angle:
            thrust_angle = self.thrust_angle[i]
        if not drag:
            drag = self.drag[i]
        if not mass:
            mass = self.mass[i]
        if not R:
            R = self.R[i]
        if not flight_heading:
            flight_heading = self.flight_heading[i]
        if not timestep:
            timestep = self.timestep

        dV = self.dVdt(thrust, thrust_angle, drag, mass, R, flight_heading)*timestep
        return dV

    def calc_dalt(self, velocity=None, flight_heading=None, timestep=None):
        """ calc_dalt is a method of Rocket

        This method is typically used to update the altitude values during
        simulation runs.

        Note:
            All arguments are optional. If no arguments are thrown,
            the method will return the calculated delta altitude
            based on the current timestep values.

        Args:
            velocity:       # [m/s] velocity at current timestep
            flight_heading: # [m] radius from center of the Earth
            timestep:       # [s] timestep

        Returns:
            dalt: change in velocity of the rocket
        """

        i = self.runIter
        if not velocity:
            velocity = self.velocity[i]
        if not flight_heading:
            flight_heading = self.flight_heading[i]
        if not timestep:
            timestep = self.timestep

        dalt = velocity*np.sin(flight_heading)*timestep
        return dalt

    def dVdt(self, thrust, thrust_angle, drag, mass, R, flight_heading):
        """ dVdt is a method of Rocket

        dVdt calculates the acceleration of the rocket at the current step
        """
        dVdt = (thrust*np.cos(thrust_angle)-drag)/mass - self.g0*(self.Rearth/R)**2*np.sin(flight_heading)
        return dVdt

    def CONST(self):
        """ Define useful constants as instance variables """
        self.g0 = 9.81          # gravity constant [m/s]
        self.R_air = 287        # gas constant [J/kg/K]
        self.gamma_air = 1.4    # ratio of specific heats
        self.Rearth = 6378000   # [m]

    # standard atmosphere model (SI units)
    def STDATM(self, altitude):
        layer = -1.0            # gradient layer
        gradient = -0.0065
        altitude_base = 0.0
        temperature_base = 288.16
        density_base = 1.2250
        
        if altitude > 11000.0:
            layer = 1.0       # isothermal layer
            altitude_base = 11000.0
            temperature_base = 216.66
            density_base = 0.3648 
        elif altitude > 25000.0:
            layer = -1.0      # gradient layer
            gradient = 0.003
            altitude_base = 25000.0
            temperature_base = 216.66
            density_base = 0.04064
        elif altitude > 47000.0:
            layer = 1.0       # isothermal layer
            altitude_base = 47000.0
            temperature_base = 282.66
            density_base = 0.001476
        elif altitude > 53000.0:
            layer = -1.0      # gradient layer
            gradient = -0.0045
            altitude_base = 53000.0
            temperature_base = 282.66
            density_base = 0.0007579
        elif altitude > 79000.0:
            layer = 1.0       # isothermal layer
            altitude_base = 79000.0
            temperature_base = 165.66
            density_base = 0.0000224    
        elif altitude > 90000.0:
            layer = -1.0      # gradient layer
            gradient = 0.004
            altitude_base = 90000.0
            temperature_base = 165.66
            density_base = 0.00000232
        if layer < 0.0:
            temperature = temperature_base + gradient*(altitude - altitude_base)
            power = -1.0*(self.g0/gradient/self.R_air + 1.0)
            density = density_base*(temperature/temperature_base)**power
        else:
            temperature = temperature_base
            power = -1.0*self.g0*(altitude - altitude_base)/self.R_air/temperature
            density = density_base*np.exp(power)
        sos = np.sqrt(self.gamma_air*self.R_air*temperature)
        
        return (temperature, density, sos)


def test_Rocket():
    duration = 10000  # [s]
    timestep = 0.1  # [s]

    gas_constant = 287  # [J/kg/K] gas constant for air
    gamma = 1.2

    # constants
    g0 = 9.81  # [m/s/s]
    Re = 6378000  # [m]

    averageTWRatio = 80

    # engine definition
    Isp = 300  # [s]
    nengines = 1
    thrust_sl = nengines*50000  # [N]
    Ae = 0.25  # [m^2]
    pe = units.Value(101325.0,'Pa')  # [atm]
    mdot = thrust_sl/(g0*Isp)  # [kg/s]
    mpropulsion = thrust_sl/averageTWRatio/g0 # [1]
    mstructure = 200  # [kg]
    mpayload = 0  # [kg]
    burntime = [150, 0]  # [s] three burns, launch, reentry, landing
    totalburn = sum(burntime)
    mprop = mdot*totalburn  # [kg]
    m0 = mstructure + mpayload + mprop + mpropulsion
    mf = m0 - mprop # [kg]
    massratio = mprop/m0
    initialThrust2Weight = thrust_sl/(m0*g0)

    print('mass ratio =', massratio)
    print('dry mass = ', mf)
    print('wet mass = ', m0)
    print('propellant mass =', mprop)
    print('propulsion system mass =', mpropulsion)
    print('initialThrust2Weight =', initialThrust2Weight)

    atm = atmos.SimpleAtmos()

    itsatest = Rocket()
    return 0


if __name__ == '__main__':
    test_Rocket()