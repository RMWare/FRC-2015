import logging
import math
from hardware.syncgroup import SyncGroup

from wpilib import DigitalInput, Encoder, Talon, Solenoid
from hardware import hardware
from . import Component
from motionplanning import TrajectoryFollower


log = logging.getLogger("elevator")


class Setpoints(object):
    BOTTOM = 0
    BIN = 14
    TOTE = 18
    FIRST_TOTE = 7


class Elevator(Component):
    ON_TARGET_DELTA = 1 / 4

    def __init__(self):
        super().__init__()
        self._motor = SyncGroup(Talon, hardware.elevator)
        self._stabilizer_piston = Solenoid(hardware.stabilizer_solenoid)

        # Motion Planning!
        self._follower = TrajectoryFollower()

        self.tote_count = 0
        self.has_bin = False  # Do we have a bin?
        self._reset = True  # starting a new stack?
        self._tote_first = False  # We're stacking totes without a bin.
        self._should_drop = False  # Are we currently trying to get a bin ?

        self._close_stabilizer = True  # Controlling the stabilizer

        self._follower.set_goal(Setpoints.BIN)  # Base state

    def stop(self):
        self._motor.set(0)

    def update(self):
        goal = self._follower.get_goal()
        if self.at_goal:
            self.do_stack_logic(goal)

        self._motor.set(self._follower.calculate(hardware.elevator_encoder.getDistance()))
        self._stabilizer_piston.set(self._close_stabilizer)
        self._tote_first = False

    def do_stack_logic(self, goal):
        if self._should_drop:  # Dropping should override everything else
            if not hardware.game_piece_in_intake():
                self._follower._max_acc = 100  # Put things down gently if there's space before the bottom tote
            else:
                self._follower._max_acc = 100000000000
            self._follower.set_goal(Setpoints.BOTTOM)
            self._close_stabilizer = False
            self._reset = True
            self._should_drop = False
            return

        self._follower._max_acc = 210  # Normal speed # TODO configurable
        if self._reset:
            self._reset = False
            self._close_stabilizer = True
        if goal == Setpoints.BOTTOM:  # If we've just gone down to grab something
            if self.tote_count == 0 and not self.has_bin and not self._tote_first:
                self.has_bin = True  # We just stacked the bin
            else:  # We just stacked a tote
                self.tote_count += 1

            self._follower.set_goal(Setpoints.TOTE)  # Go back up. After stacking, you should always grab a tote.
            if self.tote_count >= 2:
                self._close_stabilizer = True
        # If we try to stack a 6th tote it'll break the robot, don't do that.
        elif hardware.game_piece_in_intake() and self.tote_count < 6:  # We have something, go down.
            if not self.has_bin:
                # Only auto-stack totes!!
                if self._tote_first or self.tote_count > 0:
                    self._follower.set_goal(Setpoints.BOTTOM)
            else:  # We have a bin, just auto-stack.
                self._follower.set_goal(Setpoints.BOTTOM)
            if self.has_bin:  # Bin Transfer! # TODO egg Marcus to make the robot not need this.
                if self.tote_count == 1:
                    self._close_stabilizer = False
        else:  # Wait for a game piece & raise the elevator
            if self.is_empty():
                if self._tote_first:
                    self._follower.set_goal(Setpoints.FIRST_TOTE)
                else:
                    self._follower.set_goal(Setpoints.BIN)
            else:
                self._follower.set_goal(Setpoints.TOTE)

    def reset_encoder(self):
        hardware.elevator_encoder.reset()
        self._follower.set_goal(0)
        self._follower._reset = True

    def reset_stack(self):
        self._reset = True

    def at_goal(self):
        return self._follower.trajectory_finished()# and abs(self._follower.get_goal() - self.position) < 2

    def drop_stack(self):
        self._should_drop = True

    def stack_tote_first(self):
        self._tote_first = True

    def is_full(self):
        return self.tote_count == 5 and hardware.game_piece_in_intake()

    def is_empty(self):
        return self.tote_count == 0 and not self.has_bin