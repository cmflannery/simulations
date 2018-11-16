.. simulations documentation getting started file
   This file serves as a quick introduction to the command line usage of simulations.

Getting Started
===============

This document will show you how to start using `simulations`.

Installation
------------

Installing with pip::

    $ pip3 install rocket-sims

Building from source::

    $ python install -r requirements.txt
    $ python setup.py install

Simulating a Launch
-------------------

Configuration Files
~~~~~~~~~~~~~~~~~~~
openrocketengine takes a configuration file as the only input, specifying the engine propellant properties, pressures desired,
and geometric design choices. Right now, there is only one possible combination of parameters that all have to be included in
the config file. In the future, there may be additional options to automatically retrieve propellant properties from CEA.

Config files are usually named with the engine name and the revision number with a '.cfg' suffix. I.e. RBF-rev01.cfg.

A typical configuration file looks like the following::

    # This is a test configuration file for openrocketengine
    #
    # The parameters listed here are all the known parameters that openrocketengine can take as inputs.
    # Refer to the official documentation for more implementation and usage details.
    name RBF1
    units SI
    thrust 5000
    Tc 3200
    pc 2068000
    pe 101325
    MR 2.1
    MW 18.9
    gamma 2.31
    # Geometric parameters
    lstar 40
    area_ratio 5.5


Running the program
~~~~~~~~~~~~~~~~~~~
openrocketengine can be fun from the command line with the command `rocket`::

    $ rocket RBF-rev01.cfg

Outputs
~~~~~~~
openrocketengine generates an output excel workbook with two sheets; one geometric parameters, and one for performance parameters.


Recommended Workflow
~~~~~~~~~~~~~~~~~~~~
Create a directory for your enigne design files and run rocket from there. I.e.::

    ~/Documents/marginal-stability/engine/design$ rocket tsu-01.cfg

