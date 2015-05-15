import math
from common.xbox import XboxController
from wpilib import Encoder, DigitalInput, PowerDistributionPanel, Gyro


def _drive_encoder_scale():
    distance_per_rev = 4 * math.pi
    ticks_per_rev = 128
    reduction = 30 / 36
    return (distance_per_rev * reduction) / ticks_per_rev


def _elevator_encoder_scale():
    pitch_diameter = 1.432
    ticks_per_revolution = 2048
    return (pitch_diameter * math.pi) / ticks_per_revolution

drive_left_encoder = Encoder(6, 7)
drive_left_encoder.setDistancePerPulse(_drive_encoder_scale())


drive_right_encoder = Encoder(4, 5)
drive_right_encoder.setDistancePerPulse(_drive_encoder_scale())

elevator_encoder = Encoder(0, 1, True)
elevator_encoder.setDistancePerPulse(_elevator_encoder_scale())

gyro = Gyro(0)

back_photosensor = DigitalInput(8)
side_photosensor = DigitalInput(3)
pdp = PowerDistributionPanel()


# Ports for motors & stuff
drive_left = 3, 4
drive_right = 5, 6

intake_left = 0
intake_right = 7
intake_left_pdp_channel = 11
intake_right_pdp_channel = 4

elevator = 1, 2, 8

intake_solenoid = 0
stabilizer_solenoid = 1


driver = XboxController(0)
operator = XboxController(1)


# Useful stuff
def game_piece_in_intake():
    return driver.a() or not back_photosensor.get()  # photosensor should be inverted

def left_intake_current():
    pdp.getCurrent(intake_left_pdp_channel)

def right_intake_current():
    pdp.getCurrent(intake_right_pdp_channel)