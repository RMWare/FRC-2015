from components import Component


def test_components(control, fake_time, robot):
	for component in robot.components:
		assert type(component) is Component


def test_failure(control, fake_time, robot):
	for component in robot.components:
		component.fail()
		assert component.enabled is False