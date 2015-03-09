#!/usr/bin/env python3
from common.xbox import XboxController
from wpilib import SampleRobot, Timer, SmartDashboard, LiveWindow, run
from components import drive, intake, pneumatics, elevator
from common import delay, util, constants
from robotpy_ext.autonomous import AutonomousModeSelector
from robotpy_ext.common_drivers import units
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
			'drive': self.drive,
			'pneumatics': self.pneumatics,
			'intake': self.intake,
			'elevator': self.elevator
		}

		self.sd_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
		self.sd_timer.start()
		self.automodes = AutonomousModeSelector('autonomous', self.components)
		self.init_smartdashboard()

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
			throttle = -self.xbox.left_y()
			self.drive.cheesy_drive(util.deadband(self.xbox.right_x(), .1),
			                        util.deadband(throttle, .1),
			                        self.xbox.left_bumper()
			)

			self.xbox.rumble(0, 0)
			self.intake._speed = 0

			if self.xbox.right_trigger():
				self.xbox.rumble(right=0.25)
				self.elevator.intake()
				# if not self.elevator.has_tote():
				self.intake._speed = .85
			else:
				if throttle < 0 and self.elevator.has_tote():
					self.intake.spin(.85)

			if self.xbox.left_trigger():
				self.xbox.rumble(left=0.25)
				self.intake.spin(-1)

			if (not self.elevator.at_setpoint() and self.elevator.position() < units.convert(units.inch, units.tick, 16))\
					or self.xbox.right_bumper():
				self.intake.open()

			if self.xbox.right_pressed():  # slow down
				self.drive.speed_mult = 0.4
			else:
				self.drive.speed_mult = 0.75

			if self.xbox.a():
				self.intake.spin(1)

			self.elevator.failsafe_override = self.xbox.b()

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
					else:
						raise e

	def init_smartdashboard(self):
		# Tunable (Reading from SD) initial setup
		for component in self.components.values():
			c_name = type(component).__name__
			for var in component.tunables:
				SmartDashboard.getTable().putValue("%s\%s" % (c_name, var), getattr(component, var))

	def update_smartdashboard(self):
		if not self.sd_timer.hasPeriodPassed(0.2):  # we don't need to update every cycle
			return

		# Tunable (Reading from SD)
		for component in self.components.values():
			c_name = type(component).__name__
			for var in component.tunables:
				setattr(component, var, SmartDashboard.getTable().getValue("%s\%s" % (c_name, var)))

			for k, v in component.__dict__.items():
				try:
					SmartDashboard.getTable().putValue("%s\%s" % (c_name, k), v)
				except ValueError:
					pass  # lole

if __name__ == "__main__":
	run(Tachyon, physics_enabled=True)