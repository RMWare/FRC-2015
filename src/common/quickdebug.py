from networktables import NetworkTable

import logging

_tunables = []
_printables = []

log = logging.getLogger("quickdebug")


def add_tunables(object_, tunables):
	if type(tunables) is list:
		for tunable in tunables:
			_tunables.append((object_, tunable))
	else:
		_tunables.append([object_, tunables])


def add_printables(object_, printables):
	if type(printables) is list:
		for printable in printables:
			_printables.append([object_, printable])
	else:
		_printables.append([object_, printables])


def init():
	for object_, tunable in _tunables:
		object_name = type(object_).__name__
		NetworkTable.getTable(object_name).putValue(tunable, getattr(object_, tunable))


def sync():
	for object_, tunable in _tunables:
		object_name = type(object_).__name__
		default = getattr(object_, tunable)
		setattr(object_, tunable, NetworkTable.getTable(object_name).getValue(tunable, default))

	for object_, printable in _printables:
		object_name = type(object_).__name__
		if type(printable) is tuple:
			assert callable(printable[1])
			try:
				out = printable[1]()
			except BaseException as e:
				out = str(e)
			printable_name = printable[0]
		else:
			out = getattr(object_, printable)
			printable_name = printable
		try:
			NetworkTable.getTable(object_name).putValue(printable_name, out)
		except (ValueError, TypeError):  # invalid type
			try:
				NetworkTable.getTable(object_name).putValue(printable_name, repr(out))
			except ValueError as e:  # oh shit we crashed twice
				log.error(repr(e))