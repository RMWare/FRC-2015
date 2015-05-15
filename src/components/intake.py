from wpilib import Solenoid, Talon, Timer
from hardware import hardware
from . import Component

AMP_THRESHOLD = 14


class Intake(Component):
    def __init__(self):
        super().__init__()

        self._l_motor = Talon(hardware.intake_left)
        self._r_motor = Talon(hardware.intake_right)
        self._intake_piston = Solenoid(hardware.intake_solenoid)

        self._left_pwm = 0
        self._right_pwm = 0
        self._open = False

        self._align_timer = Timer()
        self._align = False

    def update(self):
        if self._align:
            self.set(-1)
        self._l_motor.set(self._left_pwm)
        self._r_motor.set(-self._right_pwm)
        self._intake_piston.set(not self._open)

    def intake_tote(self):
        if hardware.game_piece_in_intake():  # anti-bounce & slowdown
            power = 1
        else:
            power = .3 if not hardware.back_photosensor.get() else 1
        self.set(power)
        # Current-monitored anti-jam. TODO needs improvement & testing
        if hardware.left_intake_current() > AMP_THRESHOLD or hardware.right_intake_current() > AMP_THRESHOLD:
            if self._align_timer.hasPeriodPassed(0.5):
                self._align_timer.start()
                self._align = True

    def intake_bin(self):
        power = .3 if not hardware.back_photosensor.get() else .65
        self.set(power, power)

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
