#!/usr/bin/env python3
from enum import Enum
from wpilib import SampleRobot, Joystick, Timer, SmartDashboard, LiveWindow, run

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
		self.stick = Joystick(0)
		self.epos = 0

		log.info("Initializing Subsystems")

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
		self.control_loop_wait_time = 0.025
		self.automodes = AutonomousModeSelector('autonomous', self.components)
		log.info("Ready!")

	def autonomous(self):
		SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.automodes.run(self.control_loop_wait_time, iter_fn=self.update)
		Timer.delay(self.control_loop_wait_time)

	def disabled(self):
		SmartDashboard.putNumber('RobotMode', MODE_DISABLED)
		while self.isDisabled():
			self.update_smartdashboard()
			Timer.delay(0.01)

	def operatorControl(self):
		SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)
		precise_delay = delay.PreciseDelay(self.control_loop_wait_time)

		while self.isOperatorControl() and self.isEnabled():
			# Driving
			self.drive.cheesy_drive(self.stick.getRawAxis(constants.controls.right_x),
			                        -self.stick.getRawAxis(constants.controls.left_y),
			                        self.stick.getRawButton(constants.controls.left_button)
			)

			# State Machine
			if self.stick.getRawAxis(constants.controls.right_trigger) > 0.25:
				self.intake.run_intake()
			elif self.stick.getRawAxis(constants.controls.left_trigger) > 0.25:
				self.intake.run_intake_backwards()
				# self.elevator.prepare_to_stack()
			else:  # abandon
				pass
				# if self.stick.getRawButton(constants.controls.offset):
				# 	self.elevator.tote_offset()
				# if self.stick.getRawButton(1):
				# 	self.intake.extend_rails()  # rails out

			if self.elevator._state == elevator._States.PICKING_UP or self.stick.getRawButton(constants.controls.right_button):
				self.intake.open()

			# DEBUG

			if self.stick.getRawButton(1):
				self.epos -= 1
			if self.stick.getRawButton(2):
				self.epos += 1

			self.elevator.set_level(pos=elevator.units.convert(elevator.units.inch, elevator.units.tick, self.epos), force=True)

			self.update_smartdashboard()
			self.update()

			precise_delay.wait()

	def test(self):
		while self.isTest() and self.isEnabled():
			LiveWindow.run()

	def update(self):
		""" Calls the update functions for every component """
		for component in self.components.values():
			# if the component failed somehow, don't run it's update method.
			if component.enabled:
				component.update()

	def update_smartdashboard(self):
		if not self.sd_timer.hasPeriodPassed(0.2):  # we don't need to update every cycle
			return
		# log.info("Desired pos: %s, Height: %s" % (self.elevator._desired_position, self.elevator._encoder.get()))
		# log.info("hall effect: %s" % self.elevator.halleffect.get())


if __name__ == "__main__":
	run(Drake)

	# TODO TUNE PID LOOPS AND MAKE suRE IT WORKS ELEVATOOR
	# TODO abstract methods to run elevator "intake" method at arbitrary levels
	# TODO keep PID loop on when braked