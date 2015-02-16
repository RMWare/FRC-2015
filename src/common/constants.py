# This file is named misleadingly, lol


class _MotorConstants(object):  # all pins should be different
	# Drive
	drive_left = [3, 4]
	drive_right = [5, 6]

	# Intake
	intake_l = 0
	intake_r = 7

	# Elevator
	elevator_motor = [1, 2, 8]


class _SolenoidConstants(object):
	disc_brake = 0
	intake_l = 1
	intake_r = 2


class _SensorConstants(object):
	elevator_encoder_a = 0
	elevator_encoder_b = 1
	elevator_hall_effect = 2
	photosensor = 3


class _PIDConstants(object):
	kP_elevator_up = 0.0
	kI_elevator_up = 0.0
	kD_elevator_up = 0.0

	kP_elevator_down = 0.0
	kI_elevator_down = 0.0
	kD_elevator_down = 0.0


class _ControllerConstants(object):
	wheel = 4
	throttle = 1
	quickturn = 5

	stack = 0
	offset = 0
	rails = 0

motors = _MotorConstants()
pids = _PIDConstants()
solenoids = _SolenoidConstants()
sensors = _SensorConstants()
controls = _ControllerConstants()