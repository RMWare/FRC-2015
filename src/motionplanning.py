class TrajectoryFollowingPositionController(object):
	_goal = 0.0
	_error = 0.0
	_on_target_delta = 0.0
	_result = 0.0

	def __init__(self, kp, ki, kd, kv, ka, on_target_delta, config):
		self._follower = TrajectoryFollower(kp, ki, kd, kv, ka, config)
		self._on_target_delta = on_target_delta

	def setGoal(self, current_state, goal):
		self._goal = goal
		self._follower.setGoal(current_state, goal)

	def getGoal(self):
		""" generated source for method getGoal """
		return self._follower.getGoal()

	def setConfig(self, config):
		""" generated source for method setConfig """
		self._follower.setConfig(config)

	def getConfig(self):
		""" generated source for method getConfig """
		return self._follower.getConfig()

	def update(self, position, velocity):
		""" generated source for method update """
		self._error = _goal - position
		self._result = _follower.calculate(position, velocity)

	def getSetpoint(self):
		""" generated source for method getSetpoint """
		return self._follower.getCurrentSetpoint()

	def get(self):
		""" generated source for method get """
		return self._result

	def reset(self):
		""" generated source for method reset """
		self._result = 0
		self._error = 0
		self._follower.setGoal(self._follower.getCurrentSetpoint(), self._goal)

	def isOnTarget(self):
		""" generated source for method isOnTarget """
		return self._follower.isFinishedTrajectory() and Math.abs(self._error) < _on_target_delta


#
#  * PID + Feedforward controller for following a Trajectory.
#  *
#  * @author Jared341
#
class TrajectoryFollower(object):
	""" generated source for class TrajectoryFollower """
	class TrajectoryConfig(object):
		""" generated source for class TrajectoryConfig """
		dt = float()
		max_acc = float()
		max_vel = float()

		def __str__(self):
			""" generated source for method toString """
			return "dt: " + self.dt + ", max_acc: " + self.max_acc + ", max_vel: " + self.max_vel

	class TrajectorySetpoint(object):
		""" generated source for class TrajectorySetpoint """
		pos = float()
		vel = float()
		acc = float()

		def __str__(self):
			""" generated source for method toString """
			return "pos: " + self.pos + ", vel: " + self.vel + ", acc: " + self.acc

	kp_ = float()
	ki_ = float()
	kd_ = float()
	kv_ = float()
	ka_ = float()
	last_error_ = float()
	error_sum_ = float()
	reset_ = True
	last_timestamp_ = float()
	next_state_ = TrajectorySetpoint()
	config_ = TrajectoryConfig()
	goal_position_ = float()
	setpoint_ = TrajectorySetpoint()

	def __init__(self, kp, ki, kd, kv, ka, config):
		""" generated source for method configure """
		self.kp_ = kp
		self.ki_ = ki
		self.kd_ = kd
		self.kv_ = kv
		self.ka_ = ka
		self.config_ = config

	def setGoal(self, current_state, goal_position):
		""" generated source for method setGoal """
		self.goal_position_ = goal_position
		self.setpoint_ = current_state
		self.reset_ = True
		self.error_sum_ = 0.0

	def getGoal(self):
		""" generated source for method getGoal """
		return self.goal_position_

	def getConfig(self):
		""" generated source for method getConfig """
		return self.config_

	def setConfig(self, config):
		""" generated source for method setConfig """
		self.config_ = config

	def calculate(self, position, velocity):
		""" generated source for method calculate """
		dt = self.config_.dt
		if not self.reset_:
			dt = now - last_timestamp_
			self.last_timestamp_ = now
		else:
			self.last_timestamp_ = Timer.getFPGATimestamp()
		if isFinishedTrajectory():
			self.setpoint_.pos = goal_position_
			self.setpoint_.vel = 0
			self.setpoint_.acc = 0
		else:
			#  Compute the new commanded position, velocity, and acceleration.
			if distance_to_go < 0:
				inverted = True
				distance_to_go *= -1
				cur_vel *= -1
			#  Compute discriminants of the minimum and maximum reachable
			#  velocities over the remaining distance.
			if min_reachable_velocity_disc < 0 or cruise_vel < 0:
				cruise_vel = Math.min(config_.max_vel, Math.sqrt(max_reachable_velocity_disc))
			#  Accelerate
			#  to
			#  cruise_vel
			#  Decelerate
			#  to zero
			#  vel.
			#  Figure out where we should be one dt along this trajectory.
			if t_start >= dt:
				self.next_state_.pos = cur_vel * dt + 0.5 * config_.max_acc * dt * dt
				self.next_state_.vel = cur_vel + config_.max_acc * dt
				self.next_state_.acc = config_.max_acc
			elif t_start + t_cruise >= dt:
				self.next_state_.pos = x_start + cruise_vel * (dt - t_start)
				self.next_state_.vel = cruise_vel
				self.next_state_.acc = 0
			elif t_start + t_cruise + t_end >= dt:
				self.next_state_.pos = x_start + x_cruise + cruise_vel * delta_t - 0.5 * config_.max_acc * delta_t * delta_t
				self.next_state_.vel = cruise_vel - config_.max_acc * delta_t
				self.next_state_.acc = -config_.max_acc
			else:
				#  Trajectory ends this cycle.
				self.next_state_.pos = distance_to_go
				self.next_state_.vel = 0
				self.next_state_.acc = 0
			if inverted:
				self.next_state_.pos *= -1
				self.next_state_.vel *= -1
				self.next_state_.acc *= -1
			self.setpoint_.pos += next_state_.pos
			self.setpoint_.vel = next_state_.vel
			self.setpoint_.acc = next_state_.acc
		error = self.setpoint_.pos - position
		if self.reset_:
			#  Prevent jump in derivative term when we have been reset.
			self.reset_ = False
			self.last_error_ = error
			self.error_sum_ = 0
		output = self.kp_ * error + self.kd_ * ((error - self.last_error_) / dt - self.setpoint_.vel) + (self.kv_ * self.setpoint_.vel + self.ka_ * self.setpoint_.acc)
		if output < 1.0 and output > -1.0:
			#  Only integrate error if the output isn't already saturated.
			self.error_sum_ += error * dt
		output += ki_ * error_sum_
		self.last_error_ = error
		return output

	def isFinishedTrajectory(self):
		""" generated source for method isFinishedTrajectory """
		return Math.abs(self.setpoint_.pos - self.goal_position_) < 1E-3 and Math.abs(self.setpoint_.vel) < 1E-2

	def getCurrentSetpoint(self):
		""" generated source for method getCurrentSetpoint """
		return self.setpoint_


