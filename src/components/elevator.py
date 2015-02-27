from enum import Enum
import math
from wpilib import DigitalInput, Encoder, Solenoid, Talon
from common import constants, util
from common.syncgroup import SyncGroup
from robotpy_ext.common_drivers import units


class States(Enum):
	BRAKED = 0
	MOVING = 1
	ZEROING = 2
	WAITING = 3
	PICKING_UP = 4


class Elevator(object):
	pid_tolerance = units.convert(units.inch, units.tick, 1/16)
	MAX_VALUE = 26519
	MIN_VALUE = units.convert(units.inch, units.tick, 1/8)

	def __init__(self):
		self.motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self.brake = Solenoid(constants.solenoids.disc_brake)
		self.encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b, True)
		self.halleffect = DigitalInput(constants.sensors.elevator_hall_effect)
		self.photosensor = DigitalInput(constants.sensors.photosensor)

		self.prev_error = 0
		self.integral = 0
		self.desired_position = 0

		self.override_level = -1
		self.old_pos = 0
		self.offset = False

		self.state = States.ZEROING

		self.rails_extended = False

	def update(self):
		if self.state in [States.MOVING, States.PICKING_UP]:
			current_position = self.encoder.getDistance()
			curr_error = self.desired_position - current_position

			if abs(curr_error) < self.pid_tolerance:  # at setpoint
				if self.state == States.WAITING:
					if self.photosensor.get():  # tote is in robot!
						self.old_pos = self.desired_position
						self.set_level(0, force=True)
						self.state = States.PICKING_UP

				if self.state == States.PICKING_UP:  # go back to old pos
					self.set_level(pos=self.old_pos, force=True)

				elif self.state == States.MOVING:
					self.state = States.BRAKED
			else:
				derivative = curr_error - self.prev_error
				self.integral += curr_error

				self.brake.set(False)

				if curr_error > 0:
					kP = constants.pids.kP_elevator_up
					kI = constants.pids.kI_elevator_up
					kD = constants.pids.kD_elevator_up
				elif curr_error < 0:
					kP = constants.pids.kP_elevator_down
					kI = constants.pids.kI_elevator_down
					kD = constants.pids.kD_elevator_down
				else:
					kP, kI, kD = 0, 0, 0

				if kI * self.integral > 1:
					self.integral = 1 / kI
				elif kI * self.integral < -1:
					self.integral = -1 / kI

				vP = kP * curr_error
				vI = kI * self.integral
				vD = kD * derivative

				power = util.limit(vP + vI + vD)

				self.motor.set(power)
				self.prev_error = curr_error

		if self.state == States.ZEROING:
			# goes up until the hall effect sensor goes off.
			if self.encoder.getDistance() < units.convert(units.inch, units.tick, 3):
				self.motor.set(.1)
				if self.encoder.getDistance() < 0:
					self.reset_encoder()
			else:
				raise RuntimeError(
					"While trying to zero, the elevator went up more than 3 inches without triggering the hall effect sensor.")
			if self.halleffect.get():
				self.reset_encoder()
				self.state = States.BRAKED

		if self.state == States.BRAKED:
			self.motor.set(0)
			self.brake.set(True)

		self.offset = 0

	def set_level(self, level=None, pos=None, force=False):  # translates levels 0-6 into encoder value
		if self.override_level is not None and not force:
			return

		if level is None and pos is None:
			raise ValueError("set_level was passed neither a level or a position. You must pass one")
		elif level is not None and pos is not None:
			raise ValueError("set_level was passed a level and a position. Please only use one.")

		if pos is None:
			pos = min(max(self.MIN_VALUE, units.convert(units.tote, units.tick, level)), self.MAX_VALUE)

		if pos < 0:
			raise ValueError("can't set position to a value below 0.")

		if not self.desired_position == pos:
			self.state = States.MOVING
			self.desired_position = pos
			self.integral = 0

	def set_override_level(self, level):
		self.set_level(level, force=True)
		self.override_level = level

	def reset_encoder(self):
		self.encoder.reset()
		self.prev_error = 0
		self.integral = 0

	def tote_offset(self):
		self.offset = -TOTE_HEIGHT / 2

	def zero(self):
		if self.state == States.BRAKED:
			self.state = States.ZEROING

	def prepare_to_stack(self):
		if self.state == States.BRAKED:
			self.set_level(2)
			self.state = States.MOVING_TO_WAIT