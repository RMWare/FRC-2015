#!/usr/bin/env python3
from common.xbox import XboxController
from wpilib import SampleRobot, Timer, SmartDashboard, LiveWindow, run
from components import drive, intake, pneumatics, elevator
from common import delay, util, constants, quickdebug
from robotpy_ext.autonomous import AutonomousModeSelector
import logging

log = logging.getLogger("robot")

# to stay in sync with our driver station
MODE_DISABLED = 0
MODE_AUTONOMOUS = 1
MODE_TELEOPERATED = 2


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
			'drive': self.drive
			, 'pneumatics': self.pneumatics
			, 'intake': self.intake
			, 'elevator': self.elevator
		}

		self.nt_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
		self.nt_timer.start()
		self.automodes = AutonomousModeSelector('autonomous', self.components)
		quickdebug.init()

	def autonomous(self):
		SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.automodes.run(constants.general.control_loop_wait_time, iter_fn=self.update)
		Timer.delay(constants.general.control_loop_wait_time)

	def disabled(self):
		SmartDashboard.putNumber('RobotMode', MODE_DISABLED)
		while self.isDisabled():
			Timer.delay(0.01)

	def operatorControl(self):
		SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)
		precise_delay = delay.PreciseDelay(constants.general.control_loop_wait_time)
		while self.isOperatorControl() and self.isEnabled():
			# Driving
			wheel = util.deadband(self.xbox.right_x(), .15)
			throttle = -util.deadband(self.xbox.left_y(), .15)
			self.drive.cheesy_drive(wheel, throttle, self.xbox.left_bumper())
			#
			# if self.xbox.right_trigger():
			# 	# Main stacking logic follows
			# 	if self.elevator.at_setpoint():
			# 		if self.elevator.has_tote():  # going up
			# 			self.elevator.set_goal(0)  # TODO BOTTOM
			# 		else:
			# 			self.elevator.set_goal(20)  # TODO TOP
			# 	if not self.elevator.has_tote():
			# 		self.intake.spin(1)
			# else:
			# 	self.elevator.set_goal(self.elevator.NEUTRAL_POSITION)
			# else:
			# 	if throttle < 0 and self.elevator.has_tote():
			# 		self.intake.spin(.85)

			if self.xbox.left_trigger():
				self.elevator.set_goal(0)  # drop totes

			if self.xbox.right_bumper():
				self.intake.open()

			if self.xbox.right_pressed():  # slow down
				self.drive.speed_multiplier = 0.4
			else:
				self.drive.speed_multiplier = 0.8

			if self.xbox.a():
				self.intake.spin(1)

			self.elevator.seek_to_top = self.xbox.b()

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