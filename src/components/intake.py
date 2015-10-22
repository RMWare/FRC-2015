import logging
from wpilib import Solenoid, Talon, Timer
from common.util import CircularBuffer
from hardware import hardware
from . import Component

AMP_THRESHOLD = 15

log = logging.getLogger("intake")


class Intake(Component):
    def __init__(self):
        super().__init__()

        self._l_motor = Talon(hardware.intake_left)
        self._r_motor = Talon(hardware.intake_right)
        self._intake_piston = Solenoid(hardware.intake_solenoid)

        self._left_pwm = 0
        self._right_pwm = 0
        self._open = False

        self._left_intake_amp = CircularBuffer(25)
        self._right_intake_amp = CircularBuffer(25)
        self._pulse_timer = Timer()

    def update(self):
        self._l_motor.set(self._left_pwm)
        self._r_motor.set(-self._right_pwm)
        self._intake_piston.set(not self._open)
        self._left_intake_amp.append(hardware.left_intake_current())
        self._right_intake_amp.append(hardware.right_intake_current())

    def intake_tote(self):
        if hardware.game_piece_in_intake():  # anti-bounce & slowdown
            power = .8
        else:
            power = .3 if not hardware.back_photosensor.get() else 1
        self.set(power)

        if self.intake_jammed() and not self._pulse_timer.running:
            self._pulse_timer.start()

        if self._pulse_timer.running:
            self.set(-self._pulse_timer.get() / 2)
            if self._pulse_timer.get() > .05:
                self._pulse_timer.stop()
                self._pulse_timer.reset()

    def intake_bin(self):
        power = .3 if not hardware.back_photosensor.get() else .65
        self.set(power, power)

    def outtake_tote(self):
        self.set(-1, -1)

    def pause(self):
        self.set(0)

    def open(self):
        self._open = True

    def close(self):
        self._open = False

    def stop(self):
        self._l_motor.set(0)
        self._r_motor.set(0)

    def set(self, l, r=None):
        """ Spins the intakes at a given power. Pass either a power or left and right power. """
        self._left_pwm = l
        if r is None:
            self._right_pwm = l
        else:
            self._right_pwm = r

    def intake_jammed(self):
        if not hardware.back_photosensor.get():
            return False
        return self._left_intake_amp.average > AMP_THRESHOLD or self._right_intake_amp.average > AMP_THRESHOLD

    def update_nt(self):
        pass
        # log.info("left current: %s" % self._left_intake_amp.average)
        # log.info("right current: %s" % self._right_intake_amp.average)