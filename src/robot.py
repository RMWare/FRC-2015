#!/usr/bin/env python3
from enum import Enum
from wpilib import SampleRobot, Joystick, Timer, SmartDashboard, run

from components import drive, intake, pneumatics, elevator
from common import delay
from common import constants as c
from autonomous import AutonomousModeManager
import logging

log = logging.getLogger("robot")

# to stay in sync with our driver station
MODE_DISABLED = 0
MODE_AUTONOMOUS = 1
MODE_TELEOPERATED = 2


class States(Enum):
	NEUTRAL = 0
	INTAKE = 1
	ABANDON = 2


class Drake(SampleRobot):
	"""
		The heart of the robit
	"""

	def robotInit(self):
		pass  # this is just to stop wpilib from complaining

	def __init__(self):
		super().__init__()

		self.stick = Joystick(0)

		log.info("Initializing Subsystems")

		self.drive = drive.CheesyDrive()
		self.pneumatics = pneumatics.Pneumatics()
		self.intake = intake.Intake()
		self.elevator = elevator.Elevator()

		self.components = {
			'drive': self.drive,
			'pneumatics': self.pneumatics,
			'intake': self.intake,
			'elevator': self.elevator
		}

		self.sd_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
		self.sd_timer.start()
		self.control_loop_wait_time = 0.025
		self.auton_manager = AutonomousModeManager(self.components)
		log.info("Ready!")

	def autonomous(self):
		""" Called when the robot is in autonomous mode	"""

		SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.auton_manager.run(self, self.control_loop_wait_time)

	def disabled(self):
		""" Called when the robot is in disabled mode """

		SmartDashboard.putNumber('RobotMode', MODE_DISABLED)

		while self.isDisabled():
			self.update_smartdashboard()
			Timer.delay(0.01)

	def operatorControl(self):
		""" Called when the robot is in teleoperated mode """

		SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)
		precise_delay = delay.PreciseDelay(self.control_loop_wait_time)

		while self.isOperatorControl() and self.isEnabled():
			# Driving
			self.drive.move(self.stick.getRawAxis(c.controls.right_x),
			                self.stick.getRawAxis(c.controls.left_y),
			                self.stick.getRawButton(c.controls.left_button)
			)

			# State Machine
			if self.stick.getRawAxis(c.controls.right_trigger) > 0.75:
				self.intake.run_intake()
				#self.elevator.prepare_to_stack()
			else:  # abandon
				pass
			# 	if self.stick.getRawButton(c.controls.offset):
			# 		self.elevator.tote_offset()
			# 	if self.stick.getRawButton(c.controls.rails):
			# 		self.elevator.extend_rails()  # rails out

			if self.elevator.state == elevator.States.PICKING_UP or self.stick.getRawButton(c.controls.right_button):
				self.intake.open()

			self.update_smartdashboard()
			self.update()

			precise_delay.wait()

	def update(self):
		""" Calls the update functions for every component """
		for component in self.components.values():
			component.update()

	def update_smartdashboard(self):
		if not self.sd_timer.hasPeriodPassed(0.1):  # we don't need to update every cycle
			return
		pass

if __name__ == "__main__":
	run(Drake)

	# TODO turn compressor off when amp draw above threshold
	# TODO TUNE PID LOOPS AND MAKE suRE IT WORKS ELEVATOOR