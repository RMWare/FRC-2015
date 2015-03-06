#!/usr/bin/env python3
from common.xbox import XboxController
from robotpy_ext.common_drivers import units
from wpilib import SampleRobot, Timer, SmartDashboard, LiveWindow, run

from components import drive, intake, pneumatics, elevator
from common import delay, util, constants
from robotpy_ext.autonomous import AutonomousModeSelector
import logging

log = logging.getLogger("robot")
# to stay in sync with our driver station
MODE_DISABLED = 0
MODE_AUTONOMOUS = 1
MODE_TELEOPERATED = 2


class Drake(SampleRobot):
	# noinspection PyAttributeOutsideInit
	# robotInit is called straight from __init__
	def robotInit(self):
		self.xbox = XboxController(0)

		constants.init_smartdashboard()

		log.info("Initializing components")

		self.drive = drive.Drive()
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
		self.automodes = AutonomousModeSelector('autonomous', self.components)
		log.info("Ready!")

	def autonomous(self):
		SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.automodes.run(constants.general.control_loop_wait_time, iter_fn=self.update)
		Timer.delay(constants.general.control_loop_wait_time)

	def disabled(self):
		SmartDashboard.putNumber('RobotMode', MODE_DISABLED)
		while self.isDisabled():
			self.update_smartdashboard()
			Timer.delay(0.01)

	def operatorControl(self):
		SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)
		precise_delay = delay.PreciseDelay(constants.general.control_loop_wait_time)

		while self.isOperatorControl() and self.isEnabled():

			# Driving
			self.drive.cheesy_drive(self.xbox.right_x(),
			                        -self.xbox.left_y(),
			                        self.xbox.left_bumper()
			)

			# the main thinger
			if self.xbox.right_trigger():
				self.xbox.rumble(right=0.25)
				self.elevator.intaking = True
				if not self.elevator.has_tote():
					self.intake.run_intake()
			else:
				self.elevator.intaking = False

			if self.xbox.left_trigger():
				self.xbox.rumble(left=0.25)
				self.intake.run_intake_backwards()

			else:  # abandon
				self.xbox.rumble(0, 0)

			if (self.elevator.position() < units.convert(units.tick, units.tote, 1) and self.elevator.at_setpoint()) \
					or self.xbox.right_bumper():
				self.intake.open()
			self.update_smartdashboard()
			self.update()

			precise_delay.wait()

	def test(self):
		while self.isTest() and self.isEnabled():
			LiveWindow.run()

	def update(self):
		""" Calls the update functions for every component """
		for component in self.components.values():
			if component.enabled:
				try:
					component.update()
				except Exception as e:
					component.enabled = False
					component.stop()
					if self.ds.isFMSAttached():
						log.error("In subsystem %s: %s" % (component, e))
						# fail silently
					else:
						raise e

	def update_smartdashboard(self):
		if not self.sd_timer.hasPeriodPassed(0.2):  # we don't need to update every cycle
			return
		for component in self.components.values():
			component.update_smartdashboard()

		constants.update_smartdashboard()


if __name__ == "__main__":
	run(Drake)