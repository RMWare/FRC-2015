# This file is named misleadingly, lol


class _MotorConstants(object):  # all pins should be different
	# Drive
	drive_left = (3, 4)
	drive_right = (5, 6)

	# Intake
	intake_l = 0
	intake_r = 7

	# Elevator
	elevator_motor = (1, 2, 8)


class _SolenoidConstants(object):
	intake = 0
	dropper = 1


class _SensorConstants(object):
	elevator_encoder_a = 0
	elevator_encoder_b = 1
	elevator_hall_effect = 2
	intake_photosensor = 3
	stabilizer_photosensor = 4
	gyro = 0  # analog!

motors = _MotorConstants()
solenoids = _SolenoidConstants()

sensors = _SensorConstants()
