import logging
from wpilib import DigitalInput, Encoder, Talon
from common import constants, quickdebug
from common.syncgroup import SyncGroup
from robotpy_ext.common_drivers import units
from . import Component
from motionplanning import TrajectoryFollower

log = logging.getLogger("elevator")

MAX_POS = 24750
MIN_POS = 0  # units.convert(units.inch, units.tick, 1 / 8)


class Elevator(Component):
	def __init__(self):
		super().__init__()
		self.failsafe_override = False
		self._motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self._encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b, True)
		self._halleffect = DigitalInput(constants.sensors.elevator_hall_effect)
		self._photosensor = DigitalInput(constants.sensors.photosensor)

		self._on_target_delta = units.convert(units.inch, units.tick, 1 / 2)

		self._follower = TrajectoryFollower(0, 0, 0, 0, 0, TrajectoryFollower.TrajectoryConfig())
		# for moving
		self._goal = 0
		self._error = 0

		self._zeroed = False
		self._ready_to_zero = False

		self.neutral_position = 0
		quickdebug.add_tunables(self, "neutral_position")
		quickdebug.add_printables(self, [
			self._encoder.getDistance,
			self._halleffect.get,
			self._photosensor.get
		])

	def stop(self):
		self._motor.set(0)

	def update(self):
		if self.failsafe_override:
			self.set_goal(20000)
			return

		if not self._zeroed:
			# goes up until the elevator is raised off the hall effect.
			if self._ready_to_zero:
				self._motor.set(.2)
				if self._halleffect.get():
					self.reset()
					self._zeroed = True

			elif not self._halleffect.get():  # wait till we get into the hall effect first
				self._ready_to_zero = True
				self._motor.set(-.2)
			return

		self._error = self._goal - self.position()
		self._motor.set(self._follower.calculate(self.position()))

		self.set_goal(self.neutral_position)

	def set_goal(self, goal):  # translates levels 0-6 into encoder value
		self._goal = min(max(MIN_POS, goal), MAX_POS)
		self._follower.set_goal(None, goal)

	def reset(self):
		self._encoder.reset()
		self._follower.set_goal(self._follower.get_setpoint(), self._goal)

	def has_tote(self):
		return not self._photosensor.get()

	def position(self):
		return self._encoder.getDistance()

	def at_setpoint(self):
		return self._follower.trajectory_finished() and abs(self._error) < self._on_target_delta

		# Change so every component can populate a list of tunable variables that automatically get put into smartdashboard
		# TODO pip install manhole
		# >>> import manhole
		# >>> manhole.install()