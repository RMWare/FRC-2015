from math import sin, pi


def deadband(val, dead):
	"""
	:param val: value to apply deadband to
	:param dead: deadband value
	:return: 0 if deadbanded, otherwise val
	"""
	return val if abs(val) > abs(dead) else 0


def sin_scale(val, non_linearity, passes):
	"""
	recursive sin scaling! :D

	:param val: input
	:param non_linearity:
	:param passes: how many times to recurse
	:return: scaled val
	"""
	scaled = sin(pi / 2 * non_linearity * val) / sin(pi / 2 * non_linearity)
	if passes == 1:
		return scaled
	else:
		return sin_scale(scaled, non_linearity, passes=passes-1)


def limit(val, lim=1):
	return max(-lim, min(val, lim))


def wrap_accumulator(acc):
	if acc > 1:
		acc -= 1
	elif acc < -1:
		acc += 1
	else:
		acc = 0
	return acc


class MotorGroup(object):
	def __init__(self, controller, *args):  # type being a motor controller, args being numbers.
		object.__setattr__(self, 'motors', [controller(i) for i in args])
		print(self.motors)

	def __getattr__(self, item):
		return object.__getattribute__(self.motors[0], item)

	def __setattr__(self, key, value):
		for motor in self.motors:
			setattr(motor, key, value)