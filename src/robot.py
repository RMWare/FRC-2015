#!/usr/bin/env python3
import logging

from wpilib import SampleRobot, Timer, LiveWindow, run

from robotpy_ext.autonomous import AutonomousModeSelector

from common.xbox import XboxController
from common.util import deadband
from components import drive, intake, pneumatics, elevator
from common import delay, quickdebug



log = logging.getLogger("robot")

# to stay in sync with our driver station
CONTROL_LOOP_WAIT_TIME = 0.025


class Tachyon(SampleRobot):
	# noinspection PyAttributeOutsideInit
	# because robotInit is called straight from __init__
	def robotInit(self):
		self.chandler = XboxController(0)
		self.meet = XboxController(1)
		self.drive = drive.Drive()                      # So redundant omg
		self.pneumatics = pneumatics.Pneumatics()
		self.intake = intake.Intake()
		self.elevator = elevator.Elevator()

		self.components = {
			'drive': self.drive,
			'pneumatics': self.pneumatics,
			'intake': self.intake,
			'elevator': self.elevator,
		}

		self.nt_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
		self.nt_timer.start()
		self.autonomous_modes = AutonomousModeSelector('autonomous', self.components)
		quickdebug.add_printables(self, ('match_time', Timer.getMatchTime))
		quickdebug.init()

	def autonomous(self):
		self.autonomous_modes.run(CONTROL_LOOP_WAIT_TIME, iter_fn=self.update_all)
		Timer.delay(CONTROL_LOOP_WAIT_TIME)

	def update_all(self):
		self.update()
		self.update_networktables()

	def disabled(self):
		while self.isDisabled():
			self.update_networktables()
			Timer.delay(0.01)

	def operatorControl(self):
		precise_delay = delay.PreciseDelay(CONTROL_LOOP_WAIT_TIME)
		while self.isOperatorControl() and self.isEnabled():
			self.intake.prevent_bounce = self.elevator.has_game_piece and not self.elevator.full()  # loler1
			if self.meet.left_bumper() or self.elevator.has_bin:
				self.elevator.stack_tote_first()
				if self.elevator.full():
					self.intake.spin(0)
				else:
					self.intake.intake_tote()
			else:
				self.intake.intake_bin()

			self.elevator.force_stack = self.chandler.a()

			if self.chandler.right_bumper():
				self.intake.open()
			else:
				self.intake.close()

			if self.chandler.left_trigger():  # If we're trying to drop the stack
				self.intake.spin(0)
				self.intake.open()
				self.elevator.drop_stack()

			wheel = deadband(self.chandler.right_x(), .2)
			throttle = -deadband(self.chandler.left_y(), .2) * .8

			if self.chandler.right_trigger():
				wheel *= 0.3
				throttle *= 0.3

			self.drive.cheesy_drive(wheel, throttle, self.chandler.left_bumper())

			ticks = self.chandler.dpad()
			if ticks == 180:  # down on the dpad
				self.drive.set_distance_goal(-2)
			elif ticks == 0:
				self.drive.set_distance_goal(2)
			elif ticks == 90:
				self.drive.set_distance_goal(-18)

			dpad = self.meet.dpad()  # You can only call it once per loop, bcus dbouncing
			if dpad == 0 and self.elevator.tote_count < 6:
				self.elevator.add_tote()
			elif dpad == 180 and self.elevator.tote_count > 0:
				self.elevator.remove_tote()
			elif dpad == 90:
				self.elevator.set_bin(not self.elevator.has_bin)

			if self.meet.start():
				self.elevator._new_stack = True

			if self.meet.b():
				self.intake.spin(0)

			if self.meet.a():
				self.intake.spin(-1)
			self.elevator.auto_stacking = not self.meet.right_bumper()  # Disable automatic stacking if bumper pressed
			# Deadman's switch! very important for safety.
			if False and not self.ds.isFMSAttached() and not self.meet.left_trigger(): # TODO re-enable at comp
				for component in self.components.values():
					component.stop()
			else:
				self.update()

			self.update_networktables()

			precise_delay.wait()

	def test(self):
		for component in self.components.values():
			component.stop()
		while self.isTest() and self.isEnabled():
			LiveWindow.run()
			self.update_networktables()

	def update(self):
		""" Calls the update functions for every component """
		for component in self.components.values():
				try:
					component.update()
				except Exception as e:
					if self.ds.isFMSAttached():
						log.error("In subsystem %s: %s" % (component, e))
					else:
						raise

	def update_networktables(self):
		if not self.nt_timer.hasPeriodPassed(0.2):  # we don't need to update every cycle
			return
		quickdebug.sync()


if __name__ == "__main__":
	run(Tachyon)
