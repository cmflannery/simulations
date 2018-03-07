""" simulations.py
Simulations is a python based flight simulation package
for rocket and missle trajectory analysis """
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import units
import atmos


class Rocket(object):
    """ Rocket is a simulation class for rocket simulations """
    def init(self, state):
        self.state = state
        self.CONST()

    def run(self, stopTime=None, stopApogee = None):
        """ runs simulation
        
        Automatically ends simulation when the vehicle impacts
        the ground, or reaches a stable orbit. """
        while True:
            pass

        # initial state vector thrown during initialization
        mass = [m0]
        velocity = [0]
        thrust = [thrust_sl]
        altitude = [0]
        R = []
        thrust_angle = 0
        drag = [0]
        heading = np.deg2rad(90)
        dynamicPressure = [0]
        heatInput = [0]
        machNumber = [0]

        accel = [0]
        gravityterm = []

        timearray = [0]

        i = 0  # iterator
        time = 0
        while True:
            time = time + timestep
            R.append(Re+altitude[i])
            dVdt = ((thrust[i]*np.cos(thrust_angle)-drag[i])/mass[i] - g0*(Re/R[i])**2*np.sin(heading))
            dV = dVdt*timestep
            accel.append(dVdt)
            
            gravityterm.append((-g0*(Re/R[i])**2*np.sin(heading))*timestep)
            
            dalt = velocity[i]*np.sin(heading)*timestep
            altitude.append(altitude[i]+dalt)
            velocity.append(velocity[i]+dV)

            T, rho, sos = STDATM(altitude[i])
            
            M = velocity[i]/sos
            machNumber.append(abs(M))
            Cd = 0.15 + 0.6*M**2*np.exp(-M**2)
            drag.append(calc_drag(velocity[i], gamma, gas_constant, T, rho, Cd, sos))

            dynamicPressure.append(1/2*rho*velocity[i]**2)
            
            heatInput.append(1/2*dynamicPressure[i]*abs(velocity[i])*S*Cd/3)
            
            timearray.append(time)
            
            if altitude[i]<20 and time>burntime[0]:
                pa = atm.pressure(0,'m')
                thrust.append(0)
                mass.append(mass[i])
                break
            else:
                pa = atm.pressure(altitude[i],'m')
            if time < burntime[0]:
                """ Launch burn """
                thrust.append(calc_thrust(thrust_sl,Ae,pe.SIValue,pa.SIValue))
                mass.append(mass[i]-mdot*timestep)
            else:
                thrust.append(0)
                mass.append(mass[i])   
            i += 1

    def CONST(self):
        """ Define useful constants as instance variables """
        self.g0 = 9.81          # gravity constant [m/s]
        self.R_air = 287        # gas constant [J/kg/K]
        self.gamma_air = 1.4    # ratio of specific heats

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