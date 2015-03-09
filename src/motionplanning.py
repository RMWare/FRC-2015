import math
from wpilib import Timer


class TrajectoryFollower(object):
	"""
	PID + Feedforward controller for following a Trajectory.
	@author Jared341
	"""
	class TrajectoryConfig(object):
		dt = 0.1
		max_acc = 0.0
		max_vel = 0.0

		def __repr__(self):
			return "TrajectoryConfig(dt: %s, max_acc: %s, max_vel: %s)" % (self.dt, self.max_acc, self.max_vel)

	class TrajectorySetpoint(object):
		pos = 0.0
		vel = 0.0
		acc = 0.0

		def __repr__(self):
			return "TrajectoryConfig(pos: %s, vel: %s, acc: %s)" % (self.pos, self.vel, self.acc)

	_prev_error = 0.0
	_error_sum = 0.0
	_reset = True
	_last_timestamp = 0.0
	_next_state = TrajectorySetpoint()
	_goal_position = 0.0
	_setpoint = TrajectorySetpoint()

	def __init__(self, kp, ki, kd, kv, ka, config):
		self._kp = kp
		self._ki = ki
		self._kd = kd
		self._kv = kv
		self._ka = ka
		self._config = config

	def set_goal(self, current_state, goal_position):
		self._goal_position = goal_position
		#self._setpoint = current_state
		self._reset = True
		self._error_sum = 0.0

	def get_goal(self):
		return self._goal_position

	def set_config(self, config):
		self._config = config

	def calculate(self, position):
		dt = self._config.dt
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
			#  Compute the new commanded position, velocity, and acceleration.
			distance_to_go = self._goal_position - self._setpoint.pos
			cur_vel = self._setpoint.vel
			cur_vel2 = cur_vel * cur_vel
			inverted = False

			if distance_to_go < 0:
				inverted = True
				distance_to_go *= -1
				cur_vel *= -1
			#  Compute discriminants of the minimum and maximum reachable
			#  velocities over the remaining distance.
			max_reachable_velocity_disc = cur_vel2 / 2.0 + self._config.max_acc * distance_to_go
			min_reachable_velocity_disc = cur_vel2 / 2.0 - self._config.max_acc * distance_to_go
			cruise_vel = cur_vel
			if min_reachable_velocity_disc < 0 or cruise_vel < 0:
				cruise_vel = min(self._config.max_vel, math.sqrt(max_reachable_velocity_disc))

			# Accelerate to cruise velocity
			t_start = (cruise_vel - cur_vel) / self._config.max_acc
			x_start = cur_vel * t_start + .5 * self._config.max_acc * t_start * t_start

			# Deccelerate to zero velocity
			t_end = abs(cruise_vel / self._config.max_acc)
			x_end = cruise_vel * t_end - .5 * self._config.max_acc * t_end * t_end
			x_cruise = max(0, distance_to_go - x_start - x_end)
			t_cruise = abs(x_cruise / cruise_vel)

			#  Figure out where we should be one dt along this trajectory.
			if t_start >= dt:
				self._next_state.pos = cur_vel * dt + 0.5 * self._config.max_acc * dt * dt
				self._next_state.vel = cur_vel + self._config.max_acc * dt
				self._next_state.acc = self._config.max_acc
			elif t_start + t_cruise >= dt:
				self._next_state.pos = x_start + cruise_vel * (dt - t_start)
				self._next_state.vel = cruise_vel
				self._next_state.acc = 0
			elif t_start + t_cruise + t_end >= dt:
				delta_t = dt - t_start - t_cruise
				self._next_state.pos = x_start + x_cruise + cruise_vel * delta_t - 0.5 * self._config.max_acc * delta_t * delta_t
				self._next_state.vel = cruise_vel - self._config.max_acc * delta_t
				self._next_state.acc = -self._config.max_acc
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
			#  Prevent jump in derivative term when we have been reset.
			self._reset = False
			self._prev_error = error
			self._error_sum = 0
		output = self._kp * error + self._kd * ((error - self._prev_error) / dt - self._setpoint.vel) + (self._kv * self._setpoint.vel + self._ka * self._setpoint.acc)
		if 1.0 > output > -1.0:

			#  Only integrate error if the output isn't already saturated.
			self._error_sum += error * dt
		output += self._ki * self._error_sum
		self._prev_error = error
		return output

	def trajectory_finished(self):
		return abs(self._setpoint.pos - self._goal_position) < 1E-3 and abs(self._setpoint.vel) < 1E-2

	def get_setpoint(self):
		return self._setpoint


