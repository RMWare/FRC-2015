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


class SyncGroup(object):
	def __init__(self, cls, pins):
		if len(pins) == 0:
			raise ValueError("No pins provided!")
		object.__setattr__(self, "_pwm", [cls(pin) for pin in pins])

	def __getattribute__(self, item):
		ret = getattr(object.__getattribute__(self, "_pwm")[0], item)
		if hasattr(ret, "__call__"):
			return object.__getattribute__(self, "FunctionWrapper")(self, item)
		return ret

	class FunctionWrapper(object):
		def __init__(self, parent, func_name):
			self.parent = parent
			self.func_name = func_name

		def __call__(self, *args, **kwargs):
			for pwm in object.__getattribute__(self.parent, "_pwm"):
				item_func = getattr(pwm, self.func_name)
				ret = item_func(*args, **kwargs)
			return ret  # will return result of last call.

	def __setattr__(self, key, value):
		for pwm in object.__getattribute__(self, "_pwm"):
			setattr(pwm, key, value)

	def __repr__(self):
		return "PWMSyncGroup(%s)" % repr(object.__getattribute__(self, "_pwm"))