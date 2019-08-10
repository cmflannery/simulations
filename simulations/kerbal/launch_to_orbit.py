import math
import time
import krpc

conn = krpc.connect(name='Launch into orbit')
vessel = conn.space_center.active_vessel

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
# side boosters resources
sb_1_resources = vessel.resources_in_decouple_stage(stage=6, cumulative=False)
sb_1_fuel = conn.add_stream(sb_1_resources.amount, 'LiquidFuel')
sb_2_resources = vessel.resources_in_decouple_stage(stage=5, cumulative=False)
sb_2_fuel = conn.add_stream(sb_2_resources.amount, 'LiquidFuel')
sb_3_resources = vessel.resources_in_decouple_stage(stage=4, cumulative=False)
sb_3_fuel = conn.add_stream(sb_3_resources.amount, 'LiquidFuel')

# Pre-launch setup
vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

# Countdown...
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

turn_angle = 0
turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

# Main ascent loop
while True:

    # Gravity turn
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    # Separate side boosters
    if vessel.control.current_stage > 6 and sb_1_fuel() == 0:
        vessel.control.activate_next_stage()
    elif vessel.control.current_stage > 5 and sb_2_fuel() == 0:
        vessel.control.activate_next_stage()
    elif vessel.control.current_stage > 4 and sb_3_fuel() == 0:
        vessel.control.activate_next_stage()

    # Decrease throttle when approaching target apoapsis
    if apoapsis() > target_altitude*0.9:
        print('Approaching target apoapsis')
        break

# Disable engines when target apoapsis is reached
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    pass
print('Target apoapsis reached')
vessel.control.throttle = 0.0

# Wait until out of atmosphere
print('Coasting out of atmosphere')
while altitude() < 70500:
    pass
