#!/usr/bin/env python3

try:
	import wpilib
except ImportError:
	from pyfrc import wpilib

from components import drive
# TODO from components import intake and elevator
from common import delay
from autonomous import AutonomousModeManager
import logging
log = logging.getLogger("robot")

# to stay in sync with our driver station
MODE_DISABLED     = 0
MODE_AUTONOMOUS   = 1
MODE_TELEOPERATED = 2


class Drake(wpilib.SampleRobot):
	"""
		The heart of the robit
	"""

	def __init__(self):
		super().__init__()

		log.info("Robit is booted")

		# Controllers

		self.stick = wpilib.Joystick(0)

		# Motors

		self.test_motor = wpilib.Talon(0)
		self.test_motor.label = 'test_motor'

		# self.l_motor = wpilib.Talon(0)
		# self.l_motor.label = 'l_motor'
		#
		# self.r_motor = wpilib.Talon(1)
		# self.r_motor.label = 'r_motor'

		#self.robot_drive = wpilib.RobotDrive(self.l_motor, self.r_motor)

		# Sensors

		self.test_encoder = wpilib.Encoder(0, 1)
		self.gyro = None  # TODO

		"""
			Initializing components
		"""

		#self.drive = drive.Drive(self.robot_drive, self.gyro)

		self.components = {
			#'drive': self.drive,
			# 'intake': self.intake,
			# 'elevator': self.elevator,
		}

		self.sd_timer = wpilib.Timer()  # timer for smartdashboard so we don't use all our bandwidth
		self.sd_timer.start()
		self.control_loop_wait_time = 0.025
		self.autonomous = AutonomousModeManager(self.components)

	def autonomous(self):
		""" Called when the robot is in autonomous mode	"""

		wpilib.SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.autonomous.run(self, self.control_loop_wait_time)

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
				Testing stuff
			"""
			self.test_motor.set(self.stick.getRawAxis(1))

			"""
				Driving
			"""

			#self.drive.move(self.stick.getX(), self.stick.getY())

			"""
				Intake
			"""

			# blah blah blah

			"""
				Elevator
			"""

			# doop de derp

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
		if self.sd_timer.hasPeriodPassed(0.1):  # we don't need to update every cycle
			wpilib.SmartDashboard.putNumber('TestEncoder', self.test_encoder.get())

if __name__ == "__main__":
	wpilib.run(Drake)