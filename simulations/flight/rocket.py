"""Rocket defines the top-level class containig actuators and sensors"""
import numpy as np


class Rocket(object):
    """ Rocket is a simulation class for rocket simulations.

    This module performs calculations for the estimation of
    launch vehicle sizing and trajectory simulation for
    vertical launch vehicles.

    A list of assumptions, capabilities, and limitations
    will be added here as features are solidified. """
    def __init__(self,
                 time=0,
                 velocity=0,
                 flight_angle=0,
                 flight_heading=0,
                 S=None,
                 latitude=0,
                 longitude=0,
                 altitude=0,
                 mass=None,
                 heat=0,
                 Isp=None,
                 nengines=1,
                 thrust_sl=None,
                 thrust_angle=0,
                 Ae=None,
                 lift_coefficient=None,
                 bank_angle=None,
                 burn_time=None,
                 timestep=0.1):
        """ Initialization of the Rocket simulation class

        Args:
            initialConditions: expects dictionary containing
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

            burn_time: length of burn in seconds
        Keyword Args:
            timestep: (optional), timestep in seconds. Default
                timestep is 1s
        Returns:
            0: Completed with no errors

        TODO: This has a bunch of outdated information. Fix it.
        """
        # TODO: Error if one of the requied args is None
        # Initial Conditions - All initial conditions are set to 0 if no input
        # is given.

        ## Required Arguments
        self.time = [time]
        self.velocity = [velocity]
        self.flight_angle = [flight_angle]
        self.flight_heading = [np.deg2rad(flight_heading)]
        self.latitude = [latitude]
        self.longitude = [longitude]
        self.altitude = [altitude]
        self.mass = [mass]
        self.heat = [heat]
        self.S = S  # surface area

        ## Optional Arguments
        self.Isp = Isp
        self.nengines = nengines
        self.thrust_sl = thrust_sl
        self.thrust_angle = [thrust_angle]
        self.lift_coefficient = [lift_coefficient]
        self.bank_angle = [bank_angle]
        self.burn_time = burn_time
        self.timestep = timestep

        self.CONST()

    def run(self, stopTime=None, stopApogee=None):
        """ runs simulation

        Automatically ends simulation when the vehicle impacts
        the ground, or reaches a stable orbit.
        """
        # initialize arrays with values from initialConditions
        self.drag = [self.calc_Cd(0)]

        # initialize arrays with values from engines
        self.thrust = [self.thrust_sl * self.nengines]
        self.mdot = self.thrust[0] / (self.g0 * self.Isp)

        # initialize additional values
        self.acceleration = [0]
        self.R = [self.Rearth
                  ]  # [m] initial distance to the center of the earth

        self.runIter = 0  # iterator
        while True:
            self.time.append(self.time[self.runIter] + self.timestep)
            self.R.append(self.Rearth + self.altitude[self.runIter])
            T, rho, sos = self.STDATM(
                self.altitude[self.runIter])  # Thermoproperties

            M = self.velocity[self.runIter] / sos
            Cd = self.calc_Cd(M)
            self.drag.append(self.calc_drag(rho=rho, Cd=Cd))

            # calculate altitude, velocity, and acceleration
            self.altitude.append(self.altitude[self.runIter] +
                                 self.calc_dalt())
            self.velocity.append(self.velocity[self.runIter] +
                                 self.calc_deltaV())
            self.acceleration.append(self.calc_accel())

            # Thrust
            if self.time[self.runIter] <= self.burn_time:
                self.thrust.append(self.thrust[0])
                self.mass.append(self.mass[self.runIter] -
                                 self.mdot * self.timestep)
            else:
                self.thrust.append(0)
                self.mass.append(self.mass[self.runIter])
            self.flight_heading.append(
                self.flight_heading[0])  # initial values until calcs added
            self.flight_angle.append(
                self.flight_angle[0])  # initial values until calcs added
            self.thrust_angle.append(self.thrust_angle[0])

            # END CONDITIONS
            if (self.altitude[self.runIter] < 1000 and
                (self.time[self.runIter] - self.time[0]) > self.burn_time
                ) or self.time[self.runIter] > 10000:
                break

            self.runIter += 1
        return (self.altitude, self.velocity, self.acceleration, self.mass,
                self.time, self.thrust, self.drag)

    def calc_Cd(self, M):
        return 0.15 + 0.6 * M**2 * np.exp(-M**2)

    def calc_drag(self, velocity=None, rho=None, S=None, Cd=None):
        if not velocity:
            velocity = self.velocity[len(self.velocity) - 1]
        if not rho:
            rho = self.rho
        if not S:
            S = self.S
        if not Cd:
            Cd = self.Cd
        drag = 0.5 * rho * velocity**2 * S * Cd
        return drag

    def calc_thrust(self, thrust_sl=None, Ae=None, pe=None, pa=None):
        """ calc_thrust determines the thrust """

        return thrust_sl + (pe - pa) * Ae

    def calc_accel(self,
                   thrust=None,
                   thrust_angle=None,
                   drag=None,
                   mass=None,
                   g0=None,
                   R=None,
                   flight_heading=None,
                   timestep=None):
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

    def calc_deltaV(self,
                    thrust=None,
                    thrust_angle=None,
                    drag=None,
                    mass=None,
                    g0=None,
                    R=None,
                    flight_heading=None,
                    timestep=None):
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
        dV = self.dVdt(thrust, thrust_angle, drag, mass, R,
                       flight_heading) * timestep
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

        dalt = velocity * np.sin(flight_heading) * timestep
        return dalt

    def dVdt(self, thrust, thrust_angle, drag, mass, R, flight_heading):
        """ dVdt is a method of Rocket

        dVdt calculates the acceleration of the rocket at the current step
        """
        drag = drag * np.sign(self.velocity[self.runIter])
        dVdt = (thrust*np.cos(thrust_angle)-drag)/mass - \
                self.g0*(self.Rearth/R)**2*np.sin(flight_heading)
        return dVdt

    def CONST(self):
        """ Define useful constants as instance variables """
        self.g0 = 9.81  # gravity constant [m/s]
        self.R_air = 287  # gas constant [J/kg/K]
        self.gamma_air = 1.4  # ratio of specific heats
        self.Rearth = 6378000  # [m]

    # standard atmosphere model (SI units)
    def STDATM(self, altitude):
        layer = -1.0  # gradient layer
        gradient = -0.0065
        altitude_base = 0.0
        temperature_base = 288.16
        density_base = 1.2250

        if altitude > 11000.0:
            layer = 1.0  # isothermal layer
            altitude_base = 11000.0
            temperature_base = 216.66
            density_base = 0.3648
        elif altitude > 25000.0:
            layer = -1.0  # gradient layer
            gradient = 0.003
            altitude_base = 25000.0
            temperature_base = 216.66
            density_base = 0.04064
        elif altitude > 47000.0:
            layer = 1.0  # isothermal layer
            altitude_base = 47000.0
            temperature_base = 282.66
            density_base = 0.001476
        elif altitude > 53000.0:
            layer = -1.0  # gradient layer
            gradient = -0.0045
            altitude_base = 53000.0
            temperature_base = 282.66
            density_base = 0.0007579
        elif altitude > 79000.0:
            layer = 1.0  # isothermal layer
            altitude_base = 79000.0
            temperature_base = 165.66
            density_base = 0.0000224
        elif altitude > 90000.0:
            layer = -1.0  # gradient layer
            gradient = 0.004
            altitude_base = 90000.0
            temperature_base = 165.66
            density_base = 0.00000232
        if layer < 0.0:
            temperature = temperature_base + gradient * (altitude -
                                                         altitude_base)
            power = -1.0 * (self.g0 / gradient / self.R_air + 1.0)
            density = density_base * (temperature / temperature_base)**power
        else:
            temperature = temperature_base
            power = -1.0 * self.g0 * (altitude -
                                      altitude_base) / self.R_air / temperature
            density = density_base * np.exp(power)
        sos = np.sqrt(self.gamma_air * self.R_air * temperature)

        return (temperature, density, sos)


def test_Rocket():
    burn_time = 50  # s
    nengines = 1
    thrust_sl = 24000  # N
    Isp = 260
    g0 = 9.81
    mdot = nengines * thrust_sl / (g0 * Isp)
    twratio = 50  # estimated thrust 2 weight ratio
    mstructure = 300  # kg
    mpropulsion = thrust_sl / (twratio * g0)
    mpropellant = mdot * burn_time
    mass = mpropulsion + mpropellant + mstructure
    std_input = {
        'time': 0,
        'velocity': 0,
        'S': 0.24,
        'flight_angle': 0,
        'flight_heading': np.deg2rad(90),
        'latitude': 0,
        'longitude': 0,
        'altitude': 0,
        'mass': mass,
        'heat': 0,
        'lift_coefficient': 0,
        'bank_angle': 0,
        'thrust_sl': thrust_sl,
        'thrust_angle': 0,
        'Isp': Isp,
        'Ae': 0.25,
        'nengines': nengines,
        'burn_time': burn_time
    }
    itsatest = Rocket(**std_input)
    altitude, velocity, acceleration, mass, time, thrust, drag = itsatest.run()

    return altitude, velocity, acceleration, mass, time, thrust, drag
