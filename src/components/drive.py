try:
	import wpilib
except ImportError:
	from pyfrc import wpilib


class Drive(object):
	"""
		The sole interaction between the robot and its driving system
		occurs here. Anything that wants to drive the robot must go
		through this class.
	"""

	def __init__(self, robot_drive, gyro):
		"""
			Constructor.

			:param robot_drive: a `wpilib.RobotDrive` object
		"""

		# set defaults here
		self.wheel = 0
		self.throttle = 0
		self.gyro = gyro

		self.angle_constant = .040
		self.gyro_enabled = True

		self.robot_drive = robot_drive

	#
	# Verb functions -- these functions do NOT talk to motors directly. This
	# allows multiple callers in the loop to call our functions without 
	# conflicts.
	#

	def move(self, wheel, throttle):
		"""
			Causes the robot to move

			:param wheel: The speed that the robot should turn in the X direction. 1 is right [-1.0..1.0]
			:param throttle: The speed that the robot should drive in the Y direction. -1 is forward. [-1.0..1.0]
		"""

		self.wheel = wheel
		self.throttle = throttle

	def set_gyro_enabled(self, value):
		self.gyro_enabled = value

	def return_gyro_angle(self):
		return self.gyro.GetAngle()

	def reset_gyro_angle(self):
		self.gyro.Reset()

	def set_angle_constant(self, constant):
		"""Sets the constant that is used to determine the robot turning speed"""
		self.angle_constant = constant

	def angle_rotation(self, target_angle):
		"""
			Adjusts the robot so that it points at a particular angle. Returns True
			if the robot is near the target angle, False otherwise

			:param target_angle: Angle to point at, in degrees

			:returns: True if near angle, False otherwise
		"""

		if not self.gyro_enabled:
			return False

		angle_offset = target_angle - self.return_gyro_angle()

		if angle_offset < -1 or angle_offset > 1:
			self.wheel = angle_offset * self.angle_constant
			self.wheel = max(min(0.5, self.wheel), -0.5)

			return False

		return True

	#
	# Actually tells the motors to do something
	#

	def update(self):
		""" actually does stuff"""
		self.robot_drive.arcadeDrive(self.throttle, self.wheel)
		# print('wheel=%s, throttle=%s ' % (self.wheel, self.throttle))

		# by default, the robot shouldn't move
		self.wheel = 0
		self.throttle = 0
