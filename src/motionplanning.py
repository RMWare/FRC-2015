import math
from wpilib import Timer
from common import quickdebug


class TrajectoryFollower(object):
	"""
	PID + Feedforward controller for following a Trajectory.
	Originally written by Jared341 in Java
	Heavily modified.
	"""

	class TrajectorySetpoint(object):
		pos = 0.0
		vel = 0.0
		acc = 0.0

		def __repr__(self):
			return "TrajectorySetpoint(pos: %s, vel: %s, acc: %s)" % (self.pos, self.vel, self.acc)

	_prev_error = 0.0
	_error_sum = 0.0
	_reset = True
	_last_timestamp = 0.0
	_next_state = TrajectorySetpoint()
	_goal_position = 0.0
	_setpoint = TrajectorySetpoint()

	def __init__(self):
		self._kp = .35
		self._ki = .0
		self._kd = 0
		self._kv = .0
		self._ka = .0

		self._max_acc = 170
		self._max_vel = 100000000000

		quickdebug.add_tunables(self, ["_kp", "_ki", "_kd", "_kv", "_ka", "_max_acc", "_max_vel"])
		quickdebug.add_printables(self, ['_goal_position', '_error_sum', '_prev_error', '_setpoint',
		                                 ('trajectory finished?', self.trajectory_finished)])

	def set_goal(self, goal_position):
		self._goal_position = goal_position
		self._reset = True

	def get_goal(self):
		return self._goal_position

	def calculate(self, position):
		dt = .025
		if not self._reset:
			now = Timer.getFPGATimestamp()
			dt = now - self._last_timestamp
			self._last_timestamp = now
		else:
			self._last_timestamp = Timer.getFPGATimestamp()
		if self.trajectory_finished():
			self._setpoint.pos = self._goal_position
			self._setpoint.vel = 0
			self._setpoint.acc = 0
		else:
			# Compute the new commanded position, velocity, and acceleration.
			distance_to_go = self._goal_position - self._setpoint.pos
			cur_vel = self._setpoint.vel
			cur_vel2 = cur_vel * cur_vel
			inverted = False

			if distance_to_go < 0:
				inverted = True
				distance_to_go *= -1
				cur_vel *= -1
			# Compute discriminants of the minimum and maximum reachable
			# velocities over the remaining distance.
			max_reachable_velocity_disc = cur_vel2 / 2.0 + self._max_acc * distance_to_go
			min_reachable_velocity_disc = cur_vel2 / 2.0 - self._max_acc * distance_to_go
			cruise_vel = cur_vel
			if min_reachable_velocity_disc < 0 or cruise_vel < 0:
				cruise_vel = min(self._max_vel, math.sqrt(max_reachable_velocity_disc))

			# Accelerate to cruise velocity
			t_start = (cruise_vel - cur_vel) / self._max_acc
			x_start = cur_vel * t_start + .5 * self._max_acc * t_start * t_start

			# Deccelerate to zero velocity
			t_end = abs(cruise_vel / self._max_acc)
			x_end = cruise_vel * t_end - .5 * self._max_acc * t_end * t_end
			x_cruise = max(0, distance_to_go - x_start - x_end)
			t_cruise = abs(x_cruise / cruise_vel)

			#  Figure out where we should be one dt along this trajectory.
			if t_start >= dt:
				self._next_state.pos = cur_vel * dt + 0.5 * self._max_acc * dt * dt
				self._next_state.vel = cur_vel + self._max_acc * dt
				self._next_state.acc = self._max_acc
			elif t_start + t_cruise >= dt:
				self._next_state.pos = x_start + cruise_vel * (dt - t_start)
				self._next_state.vel = cruise_vel
				self._next_state.acc = 0
			elif t_start + t_cruise + t_end >= dt:
				delta_t = dt - t_start - t_cruise
				self._next_state.pos = x_start + x_cruise + cruise_vel * delta_t - 0.5 * self._max_acc * delta_t * delta_t
				self._next_state.vel = cruise_vel - self._max_acc * delta_t
				self._next_state.acc = -self._max_acc
			else:
				#  Trajectory ends this cycle.
				self._next_state.pos = distance_to_go
				self._next_state.vel = 0
				self._next_state.acc = 0
			if inverted:
				self._next_state.pos *= -1
				self._next_state.vel *= -1
				self._next_state.acc *= -1
			self._setpoint.pos += self._next_state.pos
			self._setpoint.vel = self._next_state.vel
			self._setpoint.acc = self._next_state.acc
		error = self._setpoint.pos - position
		if self._reset:
			# Prevent jump in derivative term when we have been reset_encoder.
			self._reset = False
			self._prev_error = error
			self._error_sum = 0

		output = self._kp * error + self._kd * ((error - self._prev_error) / dt - self._setpoint.vel) + (
			self._kv * self._setpoint.vel + self._ka * self._setpoint.acc)

		if 1.0 > output > -1.0:
			# Only integrate error if the output isn't already saturated.
			self._error_sum += error * dt
		output += self._ki * self._error_sum
		self._prev_error = error
		return output

	def trajectory_finished(self):
		return abs(self._setpoint.pos - self._goal_position) < 1E-3 and abs(self._setpoint.vel) < 1E-2

	def get_setpoint(self):
		return self._setpoint


