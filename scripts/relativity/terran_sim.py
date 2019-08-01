import numpy as np
from simulations.simulations.utils import configatory


def sim():
    # path relative to repo root dir
    terran_config = 'scripts/relativity/terran.yaml'
    Terran = terran_config.Vehicle(terran_config)
    print(Terran.config.sections())


if __name__ == '__main__':
    sim()
