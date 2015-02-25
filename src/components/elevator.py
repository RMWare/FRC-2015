from enum import Enum
import math
from wpilib import DigitalInput, Encoder, Solenoid, Talon
from common import constants, util
from common.syncgroup import SyncGroup

PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 128

INCHES_PER_TICK = (PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION

TOTE_HEIGHT = 12.1 * INCHES_PER_TICK

# we need a max_speed variable that's just calculated errors averaged out @ full speed


class States(Enum):
	BRAKED = 0
	MOVING = 1
	ZEROING = 2
	WAITING = 3
	PICKING_UP = 4


class Elevator(object):
	ramp_ticks = 1000  # how long it takes to go from start to finish
	max_speed = 1 * INCHES_PER_TICK  # ticks / second
	tolerance = 32

	def __init__(self):
		self.motor = SyncGroup(Talon, constants.motors.elevator_motor)
		self.brake = Solenoid(constants.solenoids.disc_brake)
		self.encoder = Encoder(constants.sensors.elevator_encoder_a, constants.sensors.elevator_encoder_b)
		self.halleffect = DigitalInput(constants.sensors.elevator_hall_effect)
		self.photosensor = DigitalInput(constants.sensors.photosensor)

		self.ramp_position = 0
		self.prev_error = 0
		self.integral = 0
		self.desired_position = 0
		
		self.override_level = -1
		self.old_pos = 0
		self.offset = False
		
		self.state = States.BRAKED
		self.set_level(1)

		self.rails_extended = False

	def update(self):
			
		if self.state in [States.MOVING, States.MOVING_TO_WAIT, States.PICKING_UP]:
			curr_height = self.encoder.getDistance()
			curr_error = self.ramp_position - curr_height
			
			if abs(curr_error) < self.tolerance:  # at setpoint
				if self.state == States.WAITING:
					if self.photosensor.get():  # tote is in robot!
						self.old_pos = self.desired_position
						self.set_level(0, force=True)
						self.state = States.PICKING_UP

				if self.state == States.PICKING_UP:  # go back to old pos
					self.set_level(pos=self.old_pos, force=True)
				elif self.state == States.MOVING:
					self.state = States.BRAKED
			else:
				derivative = curr_error - self.prev_error
				self.integral += curr_error

				self.brake.set(False)
				self.ramp_position += (self.desired_position + self.offset) / self.ramp_ticks
				self.ramp_position = util.limit(self.ramp_position, self.desired_position + self.offset)

				if curr_error > 0:
					kP = constants.pids.kP_elevator_up
					kI = constants.pids.kI_elevator_up
					kD = constants.pids.kD_elevator_up
				elif curr_error < 0:
					kP = constants.pids.kP_elevator_down
					kI = constants.pids.kI_elevator_down
					kD = constants.pids.kD_elevator_down
				else:
					kP, kI, kD = 0, 0, 0

				if kI * self.integral > 1:
					self.integral = 1 / kI
				elif kI * self.integral < -1:
					self.integral = -1 / kI

				vP = kP * curr_error
				vI = kI * self.integral
				vD = kD * derivative

				power = util.limit(vP + vI + vD)

				self.motor.set(power)
				self.prev_error = curr_error

		if self.state == States.ZEROING:
			if self.encoder.getDistance() < (3 * (1/INCHES_PER_TICK)):
				self.motor.set(-.2)
				if self.encoder.getDistance() < 0:
					self.reset_encoder()
			if self.halleffect.get():
				self.reset_encoder()
				self.state = States.BRAKED
				
		if self.state == States.BRAKED:
			self.motor.set(0)
			self.brake.set(True)
		
		self.offset = 0

	def set_level(self, level=-1, pos=-1, force=False):  # translates levels 0-6 into encoder value
		if not self.override_level == -1 and not force:
			return
		
		if level == -1 and pos == -1:
			raise ValueError("set_level was passed neither a level or a position. You must pass one")
		elif not level == -1 and not pos == -1:
			raise ValueError("set_level was passed a level and a position. Please only use one.")
		
		if pos == -1:
			pos = level * TOTE_HEIGHT * INCHES_PER_TICK
		
		if not self.desired_position == pos:
			self.state = States.MOVING
			self.desired_position = pos
			self.ramp_position = 0
			self.integral = 0
			
	def set_override_level(self, level):
		self.set_level(level, force=True)
		self.override_level = level

	def reset_encoder(self):
		self.prev_error = 0
		self.integral = 0
	
	def tote_offset(self):
		self.offset = -TOTE_HEIGHT/2

	def zero(self):
		if self.state == States.BRAKED:
			self.state = States.ZEROING

	def prepare_to_stack(self):
		if self.state == States.BRAKED:
			self.set_level(2)
			self.state = States.MOVING_TO_WAIT