from networktables import NetworkTable

_tunables = {}
_printables = {}


def add_tunables(object_, tunables):
	if type(tunables) is list:
		for tunable in tunables:
			_tunables.update({object_: tunable})
	else:
		_tunables.update({object_: tunables})


def add_printables(object_, printables):
	if type(printables) is list:
		for printable in printables:
			_printables.update({object_: printable})
	else:
		_printables.update({object_: printables})


def init():
	for object_, tunable in _tunables.items():
		object_name = type(object_).__name__
		NetworkTable.getTable(object_name).putValue(tunable, getattr(object_, tunable))


def sync():
	for object_, tunable in _tunables.items():
		object_name = type(object_).__name__
		default = getattr(object_, tunable)
		setattr(object_, tunable, NetworkTable.getTable(object_name).getValue(tunable, default))

	for object_, printable in _printables.items():
		object_name = type(object_).__name__
		if callable(printable):  # take var names as strings & references to methods
			out = printable()
			printable_name = printable.__name__
		else:
			out = getattr(object_, printable)
			printable_name = printable

		NetworkTable.getTable(object_name).putValue(printable_name, out)