import logging
import math
from wpilib import DigitalInput, Encoder, Talon, Timer
from common import constants, quickdebug
from common.syncgroup import SyncGroup
from . import Component
from motionplanning import TrajectoryFollower

log = logging.getLogger("elevator")

MAX_POS = 70  # TODO in inches
MIN_POS = 1/4

PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 2048


class Elevator(Component):
	NEUTRAL_POSITION = 0  # TODO all the constants in inches
	ELEVATOR_OFFSET = 0
	ON_TARGET_DELTA = 1/8

	def __init__(self):
		super().__init__()
		self.seek_to_top = False
		self._motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self._encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b, True)
		self._halleffect = DigitalInput(constants.sensors.elevator_hall_effect)
		self._photosensor = DigitalInput(constants.sensors.photosensor)

		self._encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

		self._follower = TrajectoryFollower()

		self._zeroed = False
		self._ready_to_zero = False

		self._goal = 0
		self._error = 0

		quickdebug.add_tunables(self, "NEUTRAL_POSITION")
		quickdebug.add_printables(self, [
			('elevator position', self._encoder.getDistance),
			('hall effect', self._halleffect.get),
			('photo sensor', self._photosensor.get),
			"_zeroed", "_ready_to_zero"
		])

	def stop(self):
		self._motor.set(0)

	def update(self):
		if not self._zeroed:
			if not self._ready_to_zero:
				self._motor.set(-.2)  # move the elevator down
				if not self._halleffect.get():  # until we've reached the hall effect sensor
					self._ready_to_zero = True
			else:  # ready to zero!
				# goes up until the elevator is raised off the hall effect.
				self._motor.set(.2)  # go up
				if self._halleffect.get():  # until we're raised off the hall effect
					self.reset()  # then reset our encoder
					self._zeroed = True
			return

		if self.seek_to_top:
			self.set_goal(30)
		else:
			self.set_goal(self.NEUTRAL_POSITION)

		self._error = self._goal - self.position()
		self._motor.set(self._follower.calculate(self.position()))

	def set_goal(self, goal):  # translates levels 0-6 into encoder value
		self._goal = min(max(MIN_POS, goal), MAX_POS)
		self._follower.set_goal(goal)

	def reset(self):
		self._encoder.reset()
		self._follower.set_goal(0)

	def has_tote(self):
		return not self._photosensor.get()

	def position(self):
		return self._encoder.getDistance()

	def at_setpoint(self):
		return self._follower.trajectory_finished() and abs(self._error) < self.ON_TARGET_DELTA

		# Change so every component can populate a list of tunable variables that automatically get put into smartdashboard
		# TODO pip install manhole
		# >>> import manhole
		# >>> manhole.install()