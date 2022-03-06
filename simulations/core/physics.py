import numpy as np


def STDATM(self, altitude):
    """Implements the standard atmosphere model in SI units"""
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
