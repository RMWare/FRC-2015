#!/usr/bin/env python3
import logging

from wpilib import SampleRobot, Timer, LiveWindow, run
from hardware import hardware  # This loads all our sensors, controllers, etc.
from components import drive, intake, pneumatics, elevator  # These must be loaded after our hardware is.
from robotpy_ext.autonomous import AutonomousModeSelector
from common.util import deadband, AutoNumberEnum
from common import delay



log = logging.getLogger("robot")

# to stay in sync with our driver station
CONTROL_LOOP_WAIT_TIME = 0.025


class States(AutoNumberEnum):
    DROPPING = ()
    STACKING = ()
    CAPPING = ()


# noinspection PyAttributeOutsideInit
class Tachyon(SampleRobot):
    # because robotInit is called straight from __init__
    def robotInit(self):
        hardware.init()  # this makes everything not break
        self.drive = drive.Drive()
        self.pneumatics = pneumatics.Pneumatics()
        self.intake = intake.Intake()
        self.elevator = elevator.Elevator()

        self.components = {
            'drive': self.drive,
            'pneumatics': self.pneumatics,
            'intake': self.intake,
            'elevator': self.elevator,
        }

        self.nt_timer = Timer()  # timer for SmartDashboard update so we don't use all our bandwidth
        self.nt_timer.start()
        self.autonomous_modes = AutonomousModeSelector('autonomous', self.components)
        self.state = States.STACKING

    def autonomous(self):
        self.autonomous_modes.run(CONTROL_LOOP_WAIT_TIME, iter_fn=self.update_all)
        Timer.delay(CONTROL_LOOP_WAIT_TIME)

    def update_all(self):
        self.update()
        self.update_nt()

    def update_nt(self):
        if self.nt_timer.hasPeriodPassed(.5):
            log.info("avg l: %s" % self.intake._left_intake_amp.average)
            log.info("avg r: %s" % self.intake._right_intake_amp.average)

    def disabled(self):
        while self.isDisabled():
            Timer.delay(0.01)
            self.update_nt()

    def operatorControl(self):
        precise_delay = delay.PreciseDelay(CONTROL_LOOP_WAIT_TIME)
        while self.isOperatorControl() and self.isEnabled():
            # States!
            if hardware.driver.left_trigger():
                self.state = States.DROPPING
            elif hardware.operator.right_trigger():
                self.state = States.CAPPING
            else:
                self.state = States.STACKING

            if self.state == States.STACKING:
                if hardware.operator.left_bumper():
                    self.elevator.stack_tote_first()

                if hardware.driver.a():
                    self.elevator.manual_stack()

                if self.elevator.is_full():
                    self.intake.pause()
                elif not self.elevator.is_empty() or self.elevator.tote_first:
                    self.intake.intake_tote()
                else:
                    self.intake.intake_bin()

                if hardware.driver.right_bumper():
                    self.intake.open()
                else:
                    self.intake.close()

            elif self.state == States.DROPPING:
                self.elevator.drop_stack()
                self.elevator.reset_stack()
                self.intake.pause()
                self.intake.open()

            wheel = deadband(hardware.driver.right_x(), .2)
            throttle = -deadband(hardware.driver.left_y(), .2)
            quickturn = hardware.driver.left_bumper()
            low_gear = hardware.driver.right_trigger()
            self.drive.cheesy_drive(wheel, throttle, quickturn, low_gear)

            driver_dpad = hardware.driver.dpad()
            if driver_dpad == 180:  # down on the dpad
                self.drive.set_distance_goal(-2)
            elif driver_dpad == 0:
                self.drive.set_distance_goal(2)
            elif driver_dpad == 90:
                self.drive.set_distance_goal(-18)

            operator_dpad = hardware.operator.dpad()  # You can only call it once per loop, bcus dbouncing
            if operator_dpad == 0 and self.elevator.tote_count < 6:
                self.elevator.tote_count += 1
            elif operator_dpad == 180 and self.elevator.tote_count > 0:
                self.elevator.tote_count -= 1
            elif operator_dpad == 90:
                self.elevator.has_bin = not self.elevator.has_bin

            if hardware.operator.start():
                self.elevator.reset_stack()

            if hardware.operator.b():
                self.intake.set(0)  # Pause?!

            if hardware.operator.a() or hardware.driver.b():
                self.intake.set(-1)

            # Deadman switch. very important for safety (at competitions).
            if not self.ds.isFMSAttached() and not hardware.operator.left_trigger() and False:  # TODO re-enable at competitions
                for component in self.components.values():
                    component.stop()
            else:
                self.update()
            self.update_nt()
            precise_delay.wait()

    def test(self):
        for component in self.components.values():
            component.stop()
        while self.isTest() and self.isEnabled():
            LiveWindow.run()

    def update(self):
        """ Calls the update functions for every component """
        for component in self.components.values():
                try:
                    component.update()
                except Exception as e:
                    if self.ds.isFMSAttached():
                        log.error("In subsystem %s: %s" % (component, e))
                    else:
                        raise

if __name__ == "__main__":
    run(Tachyon)
