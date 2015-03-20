#!/usr/bin/env python3
from common.xbox import XboxController
from wpilib import SampleRobot, Timer, SmartDashboard, LiveWindow, run

from components.drive import Drive
from components.intake import Intake
from components.pneumatics import Pneumatics
from components.elevator import Elevator
from common import delay, util, quickdebug
from robotpy_ext.autonomous import AutonomousModeSelector
import logging

log = logging.getLogger("robot")

# to stay in sync with our driver station
MODE_DISABLED = 0
MODE_AUTONOMOUS = 1
MODE_TELEOPERATED = 2

CONTROL_LOOP_WAIT_TIME = 0.025


class Tachyon(SampleRobot):
	# noinspection PyAttributeOutsideInit
	# because robotInit is called straight from __init__
	def robotInit(self):
		self.chandler = XboxController(0)
		self.meet = XboxController(1)
		self.drive = Drive()
		self.pneumatics = Pneumatics()
		self.intake = Intake()
		self.elevator = Elevator()

		self.components = {
			'drive': self.drive,
			'pneumatics': self.pneumatics,
			'intake': self.intake,
			'elevator': self.elevator,
		}

		self.nt_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
		self.nt_timer.start()
		self.automodes = AutonomousModeSelector('autonomous', self.components)
		quickdebug.init()

	def autonomous(self):
		SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.automodes.run(CONTROL_LOOP_WAIT_TIME, iter_fn=self.update)
		Timer.delay(CONTROL_LOOP_WAIT_TIME)

	def disabled(self):
		SmartDashboard.putNumber('RobotMode', MODE_DISABLED)
		while self.isDisabled():
			Timer.delay(0.01)

	def operatorControl(self):
		SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)
		precise_delay = delay.PreciseDelay(CONTROL_LOOP_WAIT_TIME)
		while self.isOperatorControl() and self.isEnabled():
			# Default closed
			self.intake.close()

			# Driving
			wheel = util.deadband(self.chandler.right_x() * .6, .15)
			throttle = -util.deadband(self.chandler.left_y(), .15)

			self.intake.spin(0)  # Default no spinnerino pls

			# Main stacking logic follows
			if self.meet.a():  # If we have 5 totes in the robot, as seen by our operator
				self.elevator.prevent_stacking()  # Don't pick up the last tote
				if self.chandler.right_trigger():  # If we're trying to intake
					self.intake.spin(1)  # Keep spinning
			elif self.chandler.right_trigger():  # Stacking mode
				# Intake, force pickup without photosensor if A button is held
				self.elevator.stack(force_stack=self.chandler.a())
				if self.meet.left_trigger():
					self.intake.spin(.75)  # And run the intakes inwards
				else:
					self.intake.spin(1)
			else:  # If we're just driving around
				self.elevator.set_goal(self.elevator.HOLD_POSITION)  # Holding height for the totes

			if self.meet.b():  # Drops the passive elevator around the bin.
				self.elevator.release_bin()

			if self.chandler.left_trigger():  # If we're trying to drop the stack
				self.elevator.set_goal(self.elevator.DROP_POSITION)
				self.intake.open()
				# if self.elevator.goal == self.elevator.DROP_POSITION and self.elevator.at_goal():
				self.elevator.release_bin()  # Drops the passive

			if self.chandler.right_bumper():  # Open & close intakes (while stacking, usually)
				self.intake.open()

			if self.chandler.right_pressed():  # Slow down?
				self.drive.cheesy_drive(wheel, throttle * 0.4, self.chandler.left_bumper())
			else:
				self.drive.cheesy_drive(wheel, throttle * 0.9, self.chandler.left_bumper())

			if self.meet.right_trigger():  # Emergency something button
				self.elevator.set_goal(30)

			if self.chandler.x():  # outtake??
				self.intake.spin(-1)

			self.update_networktables()
			self.update()

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
						raise e

	def update_networktables(self):
		if not self.nt_timer.hasPeriodPassed(0.2):  # we don't need to update every cycle
			return
		quickdebug.sync()


if __name__ == "__main__":
	run(Tachyon)