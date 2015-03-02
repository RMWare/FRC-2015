# This file is named misleadingly, lol
from wpilib import SmartDashboard


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


class _TunableConstants(object):
	def __init__(self):
		self.kP_elevator_up = .001
		self.kI_elevator_up = .0003
		self.kD_elevator_up = .001

		self.kP_elevator_down = .001
		self.kI_elevator_down = .0003
		self.kD_elevator_down = .001

		self.kP = .0009
		self.kI = .00350
		self.kD = .00002

		self.kI_limit = 30


class _GeneralConstants(object):
	control_loop_wait_time = 0.025


motors = _MotorConstants()
tunable = _TunableConstants()
solenoids = _SolenoidConstants()
sensors = _SensorConstants()
general = _GeneralConstants()


def init_smartdashboard():
	for k, v in tunable.__dict__.items():
		SmartDashboard.putNumber(k, v * 100)


def update_smartdashboard():
	for k, v in tunable.__dict__.items():
		setattr(tunable, k, SmartDashboard.getNumber(k, v) / 100)