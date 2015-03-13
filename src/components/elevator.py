import logging
import math
from wpilib import DigitalInput, Encoder, Talon, Timer
from common import constants, quickdebug
from common.syncgroup import SyncGroup
from . import Component
from motionplanning import TrajectoryFollower

log = logging.getLogger("elevator")

MAX_POS = 40
MIN_POS = 1/4

PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 2048


class Elevator(Component):
	ON_TARGET_DELTA = 1/4

	# heights n shit
	HOLD_POSITION = 0
	DROP_POSITION = 0
	STACK_POSITION = 20

	def __init__(self):
		super().__init__()
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

		self.latched = False

		quickdebug.add_tunables(self, ["HOLD_POSITION", "STACK_POSITION", "DROP_POSITION"])
		quickdebug.add_printables(self, [
			('elevator position', self._encoder.getDistance),
			('hall effect', self._halleffect.get),
			('photo sensor', self._photosensor.get),
			"_zeroed", "_ready_to_zero", "_error",
			('at setpoint', self.at_goal)
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

		self._error = self._goal - self.position()
		self._motor.set(self._follower.calculate(self.position()))

	def set_goal(self, goal):  # translates levels 0-6 into encoder value
		self._goal = min(max(MIN_POS, goal), MAX_POS)  # failsafe!
		self._follower.set_goal(goal)

	def reset(self):
		self._encoder.reset()
		self._follower.set_goal(0)

	def has_tote(self):
		return not self._photosensor.get()

	def position(self):
		return self._encoder.getDistance()

	def at_goal(self):
		return self._follower.trajectory_finished()# and abs(self._error) < self.ON_TARGET_DELTA

	def intake(self):
		if self.at_goal():
			if self.has_tote() and self._goal == self.STACK_POSITION:
				self.set_goal(self.DROP_POSITION)
			else:
				self.set_goal(self.STACK_POSITION)