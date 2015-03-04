import logging
from wpilib import DigitalInput, Encoder, Talon, SmartDashboard
from common import constants, util
from common.syncgroup import SyncGroup
from robotpy_ext.common_drivers import units
from common.util import AutoNumberEnum
from . import Component

log = logging.getLogger("elevator")


class States(AutoNumberEnum):
	TRACKING = ()
	TRACKING_TO_WAIT = ()
	WAITING = ()
	TRACKING_TO_PICKUP = ()
	ZEROING = ()


class Elevator(Component):
	pid_tolerance = units.convert(units.inch, units.tick, 1/8)
	MAX_VALUE = 26750
	MIN_VALUE = units.convert(units.inch, units.tick, 1/8)

	def __init__(self):
		super().__init__()
		self.first_tick = True
		self._motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self._encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b, True)
		self._halleffect = DigitalInput(constants.sensors.elevator_hall_effect)
		self._photosensor = DigitalInput(constants.sensors.photosensor)
		self._curr_error = 0
		self._prev_error = 0
		self._integral = 0
		self._desired_position = 0
		self._override_level = None
		self._old_pos = 0
		self._offset = False
		self.state = States.ZEROING

	def stop(self):
		self._motor.set(0)

	def update(self):
		if self.first_tick:
			if self._halleffect.get():  # safety check.
				raise RuntimeError("Elevator was not at zero position when enabled!!!")

		self._curr_error = self._desired_position - self._encoder.getDistance()

		if self.at_setpoint():  # at setpoint
			if self.state == States.TRACKING_TO_WAIT:
				self.state = States.WAITING
				return

			if self.state == States.WAITING:
				if self.has_tote():  # tote is in robot!
					self._old_pos = self._desired_position
					self.set(pos=0, forced=True)
					self.state = States.TRACKING_TO_PICKUP
					return

			if self.state == States.TRACKING_TO_PICKUP:  # go back to old pos
				self.set(pos=self._old_pos)
				return

		derivative = (self._curr_error - self._prev_error) / constants.general.control_loop_wait_time

		self._integral = util.limit(self._integral + self._curr_error * constants.general.control_loop_wait_time,
		                            constants.tunable.kI_limit)

		# if self._curr_error > 0:
		# 	kP = constants.pids.kP_elevator_up
		# 	kI = constants.pids.kI_elevator_up
		# 	kD = constants.pids.kD_elevator_up
		# elif self._curr_error < 0:
		# 	kP = constants.pids.kP_elevator_down
		# 	kI = constants.pids.kI_elevator_down
		# 	kD = constants.pids.kD_elevator_down
		# else:
		kP, kI, kD = constants.tunable.kP, constants.tunable.kI, constants.tunable.kD


		vP = kP * self._curr_error
		vI = kI * self._integral
		vD = kD * derivative

		power = util.limit(vP + vI + vD, constants.tunable.power_limit)

		self._motor.set(power)
		self._prev_error = self._curr_error

		if self.state == States.ZEROING:
			# goes up until the hall effect sensor goes off.
			if self._encoder.getDistance() < units.convert(units.inch, units.tick, 3):
				self._motor.set(.1)
				if self._encoder.getDistance() < 0:

					self.reset_encoder()
			else:
				raise RuntimeError(
					"While trying to zero, the elevator went up more than 3 inches without triggering the hall effect sensor.")
			if self._halleffect.get():
				self.reset_encoder()
				self.state = States.TRACKING
		self.first_tick = False
		self._offset = 0

	def set(self, level=None, pos=None, forced=False):  # translates levels 0-6 into encoder value
		if (self._override_level is not None and not forced) or self.state == States.ZEROING:
			return

		if level is None and pos is None:
			raise ValueError("set_level was passed neither a level or a position. You must pass one")
		elif level is not None and pos is not None:
			raise ValueError("set_level was passed a level and a position. Please only use one.")

		if pos is None:
			pos = units.convert(units.tote, units.tick, level)

		if not self._desired_position == round(pos):
			self.state = States.TRACKING
			self._desired_position = min(max(self.MIN_VALUE, round(pos)), self.MAX_VALUE)  # limiting!
			self._integral = 0

	def set_override_level(self, level):
		self.set(level, forced=True)
		self._override_level = level

	def reset_encoder(self):
		self._encoder.reset()
		self._prev_error = 0
		self._integral = 0

	def tote_offset(self):
		#self.offset = -TOTE_HEIGHT / 2
		pass

	def has_tote(self):
		return self._photosensor.get()

	def prepare_to_stack(self):
		if self.state == States.TRACKING and self.at_setpoint():
			self.set(level=2)
			self.state = States.TRACKING_TO_WAIT

	def at_setpoint(self):
		return abs(self._curr_error) < self.pid_tolerance

	def update_smartdashboard(self):
		SmartDashboard.putString("Elevator State", self.state.name)
		SmartDashboard.putNumber("Elevator Setpoint", self._desired_position)
		SmartDashboard.putString("Elevator Height", self._encoder.get())
		SmartDashboard.putBoolean("Elevator at Setpoint", self.at_setpoint())

		SmartDashboard.putNumber("Elevator Integral", self._integral)
		SmartDashboard.putNumber("Elevator Error", self._curr_error)

		if self.state == States.ZEROING:
			SmartDashboard.putBoolean("ready_to_zero", not self._halleffect.get())