#!/usr/bin/env python3

import wpilib
from wpilib import Talon, Joystick, PowerDistributionPanel, Timer, LiveWindow, IterativeRobot, Encoder, DigitalInput


class Drake(IterativeRobot):
	def robotInit(self):
		self.tal = Talon(1)
		self.stick = Joystick(0)
		self.pdp = PowerDistributionPanel()
		self.enc = Encoder(0, 1)

	def autonomousInit(self):
		self.tal.set(1)
		Timer.delay(1)
		self.tal.set(-1)
		Timer.delay(1)
		self.tal.set(0)

	def autonomousPeriodic(self):
		Timer.delay(0.04)

	def teleopInit(self):
		pass
			
	def teleopPeriodic(self):
		self.tal.set(self.stick.getY())

	def testInit(self):
		LiveWindow.setEnabled(True)

	def testPeriodic(self):
		#LiveWindow.run()
		self.tal.set(self.stick.getY())
		Timer.delay(0.1)

	def disabledInit(self):
		pass

if __name__ == "__main__":
	wpilib.run(Drake)