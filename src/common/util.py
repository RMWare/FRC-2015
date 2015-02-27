import enum
from math import sin, pi
import math
from robotpy_ext.common_drivers import units


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


class AutoNumberEnum(enum.Enum):
	def __new__(cls):
		value = len(cls.__members__) + 1
		obj = object.__new__(cls)
		obj._value_ = value
		return obj

# also fix units
PITCH_DIAMETER = 1.432
TICKS_PER_REVOLUTION = 2048
INCHES_PER_TICK = (PITCH_DIAMETER * math.pi) / TICKS_PER_REVOLUTION
units.tick = units.Unit(base_unit=units.inch, base_to_unit=lambda x: x * (1 / INCHES_PER_TICK),
                        unit_to_base=lambda x: x * INCHES_PER_TICK)
units.tote = units.Unit(base_unit=units.inch, base_to_unit=lambda x: x / 12.125, unit_to_base=lambda x: x * 12.125)

