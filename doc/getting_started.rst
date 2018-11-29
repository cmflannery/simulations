.. simulations documentation getting started file
   This file serves as a quick introduction to the command line usage of simulations.

Getting Started
===============

This a quick guide on using the :code:`simulations` library for vehicle sizing.

:code:`simulations` is build around configuration files. A config file is taken in and the ouputs
are generated. No state variables are stored beyond runtime. A config file is all you need to keep,
send to colleagues, and version control.

Inputs
------

The following is a comprehensive list of the required and optional inputs for :code:`simulations`.

REQUIRED
~~~~~~~~
* :code:`S`: 0.24,
* :code:`mass`: mass,
* :code:`thrust_sl`: thrust_sl,
* :code:`Isp`: Isp,
* :code:`Ae`: 0.25,
* :code:`nengines`: nengines,
* :code:`burn_time`: burn_time

OPTIONAL
~~~~~~~~
* :code:`velocity`: Initial Velocity :code:`(default=0)`
* :code:`thrust_angle`: 0,
* :code:`bank_angle`: 0,
* :code:`lift_coefficient`: 0,
* :code:`heat`: 0,
* :code:`altitude`: 0,
* :code:`latitude`: 0,
* :code:`longitude`: 0,
* :code:`flight_angle`: 0,
* :code:`flight_heading`: np.deg2rad(90),