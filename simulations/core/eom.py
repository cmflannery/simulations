# Equations of motion
import numpy as np


def test_sim():
    initial_conditions = {
        'v0': 0,
        'x0': 0,
        'mass': 1
    }
    x, v = sim(initial_conditions)
    print(v)
    print(x)


def sim(initial_conditions, dt=0.1, t_end=1):
    v = np.array([initial_conditions['v0']])
    x = np.array([initial_conditions['x0']])
    mass = initial_conditions['mass']

    Force = 1 # Force in the body frame

    for t in np.arange(0, t_end, dt):
        a = acceleration(Force, mass)
        print('acceleration', a)
        v = np.append(v, a * dt + v[-1])
        x = np.append(x, 1 / 2 * a * dt**2 + v[-1] * dt + x[-1])
    return x, v

def acceleration(Force, mass):
    return Force / mass


def euler_6dof(Force, Moment, ):
    """6DOF Euler Equations of Motion
    
    Parameters
    ----------
    Force: np.ndarray
        Cartesian force (Fx, Fy, Fz)
    Moment: np.ndarray
        Cartesian moment using right hand rule (Mx, My, Mz)
    
    Returns
    -------
    Ve
        Velocity in flat Earth reference frame
    Xe
        Position in flat Earth reference frame
    euler_angles
        Euler angles
    DCM
        Direction cosine matrix. Coordinate transformation from flat Earth axes
        to body-fixed axes, returned as a 3-by-3 matrix.
    Vb
        Velocity in the body-fixed frame
    omegab
        Angular rates in body-fixed axes
    domega_dt
        Angular accelerations
    Abb
        Accelerations in body-fixed axes
    Abe
        Accelerations with respect to inertial frame
    """
    pass


if __name__ == '__main__':
    test_sim()