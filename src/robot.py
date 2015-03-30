#!/usr/bin/env python3
import logging

from wpilib import SampleRobot, Timer, SmartDashboard, LiveWindow, run

from robotpy_ext.autonomous import AutonomousModeSelector

from common.xbox import XboxController
from common.util import deadband
from components import drive, intake, pneumatics, elevator, leds
from common import delay, quickdebug


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
		self.drive = drive.Drive()                      # So redundant omg
		self.pneumatics = pneumatics.Pneumatics()
		self.intake = intake.Intake()
		self.elevator = elevator.Elevator()
		# self.leds = leds.LEDStrip()

		self.components = {
			'drive': self.drive,
			'pneumatics': self.pneumatics,
			'intake': self.intake,
			'elevator': self.elevator,
		    # 'leds': self.leds,
		}

		self.nt_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
		self.nt_timer.start()
		self.autonomous_modes = AutonomousModeSelector('autonomous', self.components)
		quickdebug.init()

	def autonomous(self):
		SmartDashboard.putNumber('RobotMode', MODE_AUTONOMOUS)
		self.autonomous_modes.run(CONTROL_LOOP_WAIT_TIME, iter_fn=self.update_all)
		Timer.delay(CONTROL_LOOP_WAIT_TIME)

	def update_all(self):
		self.update()
		self.update_networktables()

	def disabled(self):
		SmartDashboard.putNumber('RobotMode', MODE_DISABLED)
		while self.isDisabled():
			self.update_networktables()
			Timer.delay(0.01)

	def operatorControl(self):
		SmartDashboard.putNumber('RobotMode', MODE_TELEOPERATED)
		precise_delay = delay.PreciseDelay(CONTROL_LOOP_WAIT_TIME)
		while self.isOperatorControl() and self.isEnabled():

			if self.chandler.right_trigger():
				self.elevator.stack_tote_first()
				self.intake.close()
				self.intake.spin(1)
			else:
				if not self.elevator.has_bin:
					self.intake.spin(.75)  # Intaking woo
					if self.elevator.tote_count == 0:
						self.intake.open()  # Only open for the first tote
				else:  # If we have a bin, then just intake
					self.intake.spin(0 if self.elevator.full() else 1)
					if self.elevator.at_goal:
						self.intake.close()

			self.elevator._force_stack = self.chandler.a()

			if self.chandler.left_trigger():  # If we're trying to drop the stack
				self.intake.spin(0)
				self.intake.open()
				self.elevator.drop_stack()

			if self.chandler.right_bumper():
				if not self.elevator.has_bin and self.elevator.tote_count == 0:
					self.intake.close()
				else:
					self.intake.open()

			wheel = deadband(self.chandler.right_x(), .2)
			throttle = -deadband(self.chandler.left_y(), .23) * 0.8

			if self.chandler.right_pressed():
				throttle *= 0.4
				wheel *= 0.4

			self.drive.cheesy_drive(wheel, throttle * 0.75, self.chandler.left_bumper())

			if self.meet.dpad() == 0:
				self.elevator._tote_count += 1
			elif self.meet.dpad() == 180:
				self.elevator._tote_count -= 1
			elif self.meet.dpad() == 90:
				self.elevator._has_bin = not self.elevator._has_bin

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
						raise e

	def update_networktables(self):
		if not self.nt_timer.hasPeriodPassed(0.2):  # we don't need to update every cycle
			return
		quickdebug.sync()


if __name__ == "__main__":
	run(Tachyon)
