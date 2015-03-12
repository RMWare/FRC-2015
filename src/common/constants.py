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
	outtake = 1


class _SensorConstants(object):
	elevator_encoder_a = 0
	elevator_encoder_b = 1
	elevator_hall_effect = 2
	photosensor = 3
	gyro = 0  # analog!


class _GeneralConstants(object):
	control_loop_wait_time = 0.025


motors = _MotorConstants()
solenoids = _SolenoidConstants()

sensors = _SensorConstants()
general = _GeneralConstants()