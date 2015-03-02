from pyfrc.tests import *
from robotpy_ext.autonomous.selector_tests import *

from components import Component


def test_components(control, fake_time, robot):
	robot.robotInit()
	for component in robot.components.values():
		assert type(component).__bases__[0] is Component


def test_failure(control, fake_time, robot):
	robot.robotInit()
	for component in robot.components.values():
		component.enabled = False
		component.stop()

	#assert robot.  # TODO assert that all PWM outputs are set to 0