#!/usr/bin/env python3
from common.xbox import XboxController
from wpilib import SampleRobot, Timer, SmartDashboard, LiveWindow, run
from components import drive, intake, pneumatics, elevator
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
		self.xbox = XboxController(0)
		self.drive = drive.Drive()
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
			# Driving
			wheel = util.deadband(self.xbox.right_x() * .6, .15)
			throttle = -util.deadband(self.xbox.left_y(), .15)

			# Main stacking logic follows

			if self.xbox.right_trigger():
				self.elevator.intake(force_pickup=self.xbox.a())
				self.intake.spin(1)
			else:
				self.intake.spin(0)
				self.elevator.set_goal(self.elevator.HOLD_POSITION)  # default hold

			if self.xbox.left_trigger():
				self.elevator.set_goal(self.elevator.DROP_POSITION)  # release_bin totes
				self.intake.open()
				self.elevator.release_bin()

			if self.xbox.right_pressed():  # slow down
				self.drive.cheesy_drive(wheel, throttle * 0.4, self.xbox.left_bumper())
			else:
				self.drive.cheesy_drive(wheel, throttle * 0.8, self.xbox.left_bumper())

			if self.xbox.b():
				self.elevator.set_goal(30)

			if self.xbox.x():
				self.intake.spin(-1)

			if self.xbox.y():
				self.intake.spin(1, same_direction=True)

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
	run(Tachyon, physics_enabled=True)