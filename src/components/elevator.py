import logging
from wpilib import DigitalInput, Encoder, Talon
from common import constants, util
from common.syncgroup import SyncGroup
from robotpy_ext.common_drivers import units
from common.util import AutoNumberEnum

log = logging.getLogger("elevator")


class _States(AutoNumberEnum):
	TRACKING = ()
	WAITING = ()
	PICKING_UP = ()
	ZEROING = ()


class Elevator(object):
	pid_tolerance = units.convert(units.inch, units.tick, 1/16)
	MAX_VALUE = 26750
	MIN_VALUE = units.convert(units.inch, units.tick, 1/8)

	def __init__(self):
		self.enabled = True

		self._motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self._encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b, True)
		self._halleffect = DigitalInput(constants.sensors.elevator_hall_effect)
		self._photosensor = DigitalInput(constants.sensors.photosensor)

		self._curr_error = 0
		self._prev_error = 0
		self._integral = 0
		self._desired_position = 0

		self._override_level = -1
		self._old_pos = 0
		self._offset = False

		self._state = _States.ZEROING

		self._rails_extended = False

		# Safety check!
		# if self._halleffect.get():  # if we're not at the very bottom of our lift at the start of the match
		# 	log.error("Elevator was not zeroed, disabling elevator.")
		# 	self.fail()

	def update(self):
		self._curr_error = self._desired_position - self._encoder.getDistance()

		if self._state in [_States.TRACKING, _States.PICKING_UP]:

			if self.at_setpoint():  # at setpoint
				if self._state == _States.WAITING:
					if self._photosensor.get():  # tote is in robot!
						self._old_pos = self._desired_position
						self.set_level(0, force=True)
						self._state = _States.PICKING_UP

				if self._state == _States.PICKING_UP:  # go back to old pos
					self.set_level(pos=self._old_pos, force=True)
			else:
				derivative = self._curr_error - self._prev_error
				self._integral += self._curr_error

				if self._curr_error > 0:
					kP = constants.pids.kP_elevator_up
					kI = constants.pids.kI_elevator_up
					kD = constants.pids.kD_elevator_up
				elif self._curr_error < 0:
					kP = constants.pids.kP_elevator_down
					kI = constants.pids.kI_elevator_down
					kD = constants.pids.kD_elevator_down
				else:
					kP, kI, kD = 0, 0, 0

				if kI * self._integral > 1:
					self._integral = 1 / kI
				elif kI * self._integral < -1:
					self._integral = -1 / kI

				vP = kP * self._curr_error
				vI = kI * self._integral
				vD = kD * derivative

				power = util.limit(vP + vI + vD, lim=0.75)

				self._motor.set(power)
				self._prev_error = self._curr_error

		if self._state == _States.ZEROING:
			# goes up until the hall effect sensor goes off.
			if self._encoder.getDistance() < units.convert(units.inch, units.tick, 3):
				self._motor.set(.075)
				if self._encoder.getDistance() < 0:
					self.reset_encoder()
			else:
				raise RuntimeError(
					"While trying to zero, the elevator went up more than 3 inches without triggering the hall effect sensor.")
			if self._halleffect.get():
				self.reset_encoder()
				self._state = _States.TRACKING

		self._offset = 0

	def set_level(self, level=None, pos=None, force=False):  # translates levels 0-6 into encoder value
		if self._override_level is not None and not force:
			return

		if level is None and pos is None:
			raise ValueError("set_level was passed neither a level or a position. You must pass one")
		elif level is not None and pos is not None:
			raise ValueError("set_level was passed a level and a position. Please only use one.")

		if pos is None:
			pos = units.convert(units.tote, units.tick, level)

		if not self._desired_position == pos:
			self._state = _States.TRACKING
			self._desired_position = min(max(self.MIN_VALUE, pos), self.MAX_VALUE)  # limiting!
			self._integral = 0

	def set_override_level(self, level):
		self.set_level(level, force=True)
		self._override_level = level

	def reset_encoder(self):
		self._encoder.reset()
		self._prev_error = 0
		self._integral = 0

	def tote_offset(self):
		#self.offset = -TOTE_HEIGHT / 2
		pass

	def prepare_to_stack(self):
		if self._state == _States.TRACKING and self.at_setpoint():
			self.set_level(2)
			self._state = _States.MOVING_TO_WAIT

	def at_setpoint(self):
		return abs(self._curr_error) < self.pid_tolerance

	def fail(self):
		"""
		Disables EVERYTHING. Only use in case of critical failure/
		:return:
		"""
		self.enabled = False
		self._motor.set(0)