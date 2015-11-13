import enum
from math import sin, pi
from collections import deque


def deadband(val, dead):
    """
    :param val: value to apply deadband to
    :param dead: deadband value
    :return: 0 if value < dead, else val
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
        return sin_scale(scaled, non_linearity, passes - 1)


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


class CircularBuffer(deque):
    def __init__(self, size=0):
        super().__init__(maxlen=size)

    @property
    def average(self):
        try:
            return sum(self) / len(self)
        except ZeroDivisionError:
            return 0