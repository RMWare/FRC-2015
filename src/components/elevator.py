import logging
import math

from wpilib import DigitalInput, Encoder, Talon, Solenoid

from common import constants, quickdebug
from common.syncgroup import SyncGroup
from . import Component
from motionplanning import TrajectoryFollower


log = logging.getLogger("elevator")

PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 2048


class Setpoints(object):
	BOTTOM = 1
	BIN = 11
	TOTE = 18
	FIRST_TOTE = 18


class Elevator(Component):
	ON_TARGET_DELTA = 1 / 4

	def __init__(self):
		super().__init__()
		self._motor = SyncGroup(Talon, constants.motor_elevator)
		self._position_encoder = Encoder(*constants.encoder_elevator)
		self._intake_photosensor = DigitalInput(constants.intake_photosensor)
		self._dropper_piston = Solenoid(constants.solenoid_dropper)
		self._position_encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

		# Trajectory controlling stuff
		self._follower = TrajectoryFollower()

		self._tote_count = 0  # Keep track of totes!
		self._has_bin = False  # Do we have a bin on top?

		self._tote_first = False  # Override bin first to grab totes before anything else
		self._should_drop = False  # Are we currently trying to get a bin ?

		self._should_open_stabilizer = False  # Opens the stabilizer manually
		self._force_stack = False  # manually actuates the elevator down and up

		quickdebug.add_tunables(Setpoints, ["BOTTOM", "BIN", "TOTE", "FIRST_TOTE"])
		quickdebug.add_printables(self, [
			('position', self._position_encoder.getDistance),
			('photosensor', self._intake_photosensor.get),
			('at goal', self.at_goal),
			"_has_bin", "_tote_count", "_tote_first"
		])

	def stop(self):
		self._motor.set(0)

	def update(self):
		goal = self._follower.get_goal()

		if self.at_goal():
			if self._should_drop:  # Overrides everything else
				self._follower.set_goal(Setpoints.BOTTOM)
				self._dropper_piston.set(False)
			else:
				if goal == Setpoints.BOTTOM:  # If we've just gone down to grab something
					if self._tote_count == 0 and not self._has_bin and not self._tote_first:
						self._has_bin = True  # Count the bin
					else:  # We were waiting for a tote
						self._tote_count += 1
					self._follower.set_goal(Setpoints.TOTE)  # Go back up
				elif self.has_game_piece() and self._tote_count < 5:  # If we try to stack a 6th tote it'll break the robot
					self._follower.set_goal(Setpoints.BOTTOM)
					if self._has_bin:  # Transfer!
						if self._tote_count == 1:
							self._dropper_piston.set(False)
						elif self._tote_count == 2:
							self._dropper_piston.set(True)
				else:  # Wait for a game piece & raise the elevator
					if self._tote_count == 0:
						if self._tote_first:
							self._follower.set_goal(Setpoints.FIRST_TOTE)
						else:
							self._follower.set_goal(Setpoints.BIN)
					else:
						self._follower.set_goal(Setpoints.TOTE)

		self._motor.set(self._follower.calculate(self.position()))
		self._should_drop = False
		self._tote_first = False

	def reset_encoder(self):
		self._position_encoder.reset()
		self._follower.set_goal(0)
		self._follower._reset = True

	def has_game_piece(self):
		return not self._intake_photosensor.get() or self._force_stack

	def position(self):
		return self._position_encoder.getDistance()

	def at_goal(self):
		return self._follower.trajectory_finished()

	def drop_stack(self):
		self._should_drop = True
		self._tote_count = 0
		self._has_bin = False

	def tote_first(self):
		self._tote_first = True

	def full(self):
		return self._tote_count == 5 and self.has_game_piece()
