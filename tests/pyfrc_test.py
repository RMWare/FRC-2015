from pyfrc.tests import *
from robotpy_ext.autonomous.selector_tests import *

from components import Component


def test_components(control, fake_time, robot):
	robot.robotInit()
	for component in robot.components.values():
		try:
			assert type(component).__bases__[0] is Component
		except AssertionError:
			raise AssertionError("%s is not a Component" % component)


def test_failure(control, fake_time, robot):
	robot.robotInit()
	for component in robot.components.values():
		component.fail()
		assert component.enabled is False