import logging
import math
from threading import Thread
import time

from wpilib import DigitalInput, Encoder, Talon, Solenoid

from common import constants, quickdebug
from common.util import CircularBuffer
from common.syncgroup import SyncGroup
from . import Component
from motionplanning import TrajectoryFollower


log = logging.getLogger("elevator")

PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 2048


class Setpoints(object):
    DROP = 0
    STACK = 0.1
    BIN = 14
    TOTE = 18
    FIRST_TOTE = 7
    AUTON = 21


class Elevator(Component):
    ON_TARGET_DELTA = 1 / 4

    def __init__(self):
        super().__init__()
        self._motor = SyncGroup(Talon, constants.motor_elevator)
        self._position_encoder = Encoder(*constants.encoder_elevator)
        self._photosensor = DigitalInput(constants.photosensor)
        self._stabilizer_piston = Solenoid(constants.solenoid_dropper)
        self._position_encoder.setDistancePerPulse((PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION)

        # Trajectory controlling stuff
        self._follower = TrajectoryFollower()
        self.assure_tote = CircularBuffer(5)

        self.auto_stacking = True  # Do the dew

        self._tote_count = 0  # Keep track of totes!
        self._has_bin = False  # Do we have a bin on top?
        self._new_stack = True  # starting a new stack?
        self._tote_first = False  # Override bin first to grab totes before anything else
        self._should_drop = False  # Are we currently trying to get a bin ?

        self._close_stabilizer = True  # Opens the stabilizer manually
        self.force_stack = False  # manually actuates the elevator down and up

        self._follower.set_goal(Setpoints.BIN)  # Base state
        self._follower_thread = Thread(target=self.update_follower)
        self._follower_thread.start()

        self._auton = False

        quickdebug.add_tunables(Setpoints, ["DROP", "STACK", "BIN", "TOTE", "FIRST_TOTE"])
        quickdebug.add_printables(self, [
            ('position', self._position_encoder.getDistance),
            ('photosensor', self._photosensor.get),
            "has_bin", "tote_count", "tote_first", "at_goal", "has_game_piece", "auto_stacking"
        ])

    def stop(self):
        self._motor.set(0)

    def update(self):
        goal = self._follower.get_goal()
        if self.at_goal:
            self.assure_tote.append(self.has_game_piece)
            if self._should_drop:  # Overrides everything else
                if self.tote_count < 5 and not self.has_game_piece:
                    self._follower._max_acc = 100  # Slow down on drop
                self._follower.set_goal(Setpoints.DROP)
                self._close_stabilizer = False
                self._new_stack = True
            else:
                self._follower._max_acc = 210  # Normal speed
                if self._new_stack:
                    self._new_stack = False
                    self._close_stabilizer = True
                    self._tote_count = 0
                    self._has_bin = False
                if goal == Setpoints.STACK:  # If we've just gone down to grab something
                    if self.tote_count == 0 and not self.has_bin and not self.tote_first:
                        self._has_bin = True  # Count the bin
                    else:  # We were waiting for a tote
                        self.add_tote()
                    self._follower.set_goal(Setpoints.AUTON if self._auton else Setpoints.TOTE)  # Go back up
                    if self._tote_count >= 2:
                        self._close_stabilizer = True
                elif (self.tote_in_long_enough() and self.auto_stacking) and self.tote_count < 5:  # If we try to stack a 6th tote it'll break the robot
                    if not self.has_bin:
                        # Without a bin, only stack in these situations:
                        # - We're picking up a tote, and this is the first tote.
                        # - We're forcing the elevator to stack (for the bin, usually)
                        # - We have more than one tote already, so we can pick up more.
                        if self.tote_first or self.force_stack or self.tote_count > 0:
                            self._follower.set_goal(Setpoints.STACK)
                    else:  # We have a bin, just auto-stack.
                        self._follower.set_goal(Setpoints.STACK)
                    if self.has_bin:  # Transfer!
                        if self._tote_count == 1:
                            self._close_stabilizer = False
                else:  # Wait for a game piece & raise the elevator
                    if self._auton:
                        self._follower.set_goal(Setpoints.AUTON)
                    elif self._tote_count == 0 and not self.has_bin:
                        if self.tote_first:
                            self._follower.set_goal(Setpoints.FIRST_TOTE)
                        else:
                            self._follower.set_goal(Setpoints.BIN)
                    else:
                        self._follower.set_goal(Setpoints.TOTE)

        self._motor.set(self._follower.output)
        self._stabilizer_piston.set(self._close_stabilizer)
        self._should_drop = False
        self._tote_first = False
        self._auton = False

    def reset_encoder(self):
        self._position_encoder.reset()
        self._follower.set_goal(0)
        self._follower._reset = True

    @property
    def has_game_piece(self):
        return not self._photosensor.get() or self.force_stack

    @property
    def position(self):
        return self._position_encoder.getDistance()

    @property
    def at_goal(self):
        return self._follower.trajectory_finished()  # and abs(self._follower.get_goal() - self.position) < 2

    def drop_stack(self):
        self._should_drop = True

    def stack_tote_first(self):
        self._tote_first = True

    def full(self):
        return self._tote_count == 5 and self.has_game_piece

    @property
    def has_bin(self):
        return self._has_bin

    @property
    def tote_first(self):
        return self._tote_first

    @property
    def tote_count(self):
        return self._tote_count

    def add_tote(self, number=1):
        self._tote_count = max(0, min(5, self._tote_count + number))

    def remove_tote(self):
        self.add_tote(-1)

    def set_bin(self, bin_):
        self._has_bin = bin_

    def auton(self):
        self._auton = True
        self._tote_first = True

    def tote_in_long_enough(self):
        return all(self.assure_tote)

    def update_follower(self):
        while True:
            self._follower.calculate(self.position)
            time.sleep(0.005)