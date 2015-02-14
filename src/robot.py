#!/usr/bin/env python3
from enum import Enum

try:
	import wpilib
except ImportError:
	from pyfrc import wpilib

from components import drive, intake, elevator
from common import delay
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


class Drake(wpilib.SampleRobot):
	"""
		The heart of the robit
	"""

	def robotInit(self):
		pass  # this is just to stop wpilib from complaining

	def __init__(self):
		super().__init__()

		log.info("Robit is booted")

		# Controllers

		self.stick = wpilib.Joystick(0)

		"""
			Initializing components
		"""

		self.drive = drive.Drive()
		self.intake = intake.Intake()
		self.elevator = elevator.Elevator()
		self.components = {'drive': self.drive, 'intake': self.intake, 'elevator': self.elevator}

		self.sd_timer = wpilib.Timer()  # timer for smartdashboard so we don't use all our bandwidth
		self.sd_timer.start()
		self.control_loop_wait_time = 0.025
		self.auton_manager = AutonomousModeManager(self.components)

	def autonomous(self):
		""" Called when the robot is in autonomous mode	"""

		wpilib.SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.auton_manager.run(self, self.control_loop_wait_time)

	def disabled(self):
		""" Called when the robot is in disabled mode """

		wpilib.SmartDashboard.putNumber('RobotMode', MODE_DISABLED)

		while self.isDisabled():
			self.update_smartdashboard()
			wpilib.Timer.delay(0.01)

	def operatorControl(self):
		""" Called when the robot is in teleoperated mode """

		wpilib.SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)

		precise_delay = delay.PreciseDelay(self.control_loop_wait_time)

		while self.isOperatorControl() and self.isEnabled():

			"""
				Driving
			"""

			self.drive.move(self.stick.getX(), self.stick.getY(), self.stick.getRawButton(1)) # TODO BUTTON

			"""
				State Machine
			"""

			if self.stick.getRawButton(2):  # TODO button
				self.intake.intaking = True
				self.elevator.prepare_to_stack()
			else:  # abandon
				if self.stick.getRawButton(3):  # TODO button
					self.elevator.tote_offset()
				if self.stick.getRawButton(4):  # TODO
					pass  # rails out
				elif self.stick.getRawButton(5):  # TODO
					pass  # rails in

			"""
				Misc.
			"""

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


		# wpilib.SmartDashboard.putNumber('TestEncoder', self.test_encoder.get())

if __name__ == "__main__":
	wpilib.run(Drake)