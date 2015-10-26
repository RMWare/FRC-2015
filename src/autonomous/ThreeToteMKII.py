'''
Three tote auto without the broken StatefulAutonomous
'''
from . import TachyAutonomous
import logging

log = logging.getLogger('robot')

FIRST_IN_SEQUENCE = 'get_first_tote'
END = 'end'

class ThreeToteMKII(TachyAutonomous):

    def __init__(self, components):
        super().__init__(components, FIRST_IN_SEQUENCE, END)
        #Enable if you want to use timed states
        #self.timer = 0
        #self.time_goal = 0

    #Iterates through every control loop
    def iterate(self, time):
        log.info('iterating')
        '''
        #Un-comment if you want to use timed states
        #To implement a timer as the goal, simply change the timer variable to anything above 0

        if self.time_goal > 0:
            self.goal = self.timer >= self.time_goal
            if self.goal:
                self.timer = 0
                self.time_goal = 0;
            else:
                self.timer += time
        '''
        if self.drive.driving_distance:
            self.drive.drive_distance()
            self.goal = self.drive.at_distance_goal()
        elif self.drive.driving_angle:
            self.drive.drive_angle()
            self.goal = self.drive.at_angle_goal()
        else:
            self.goal = False
        super().iterate()

    #All states. Returns name of next state.
    def get_first_tote(self):
        self.drive.set_distance_goal(15)
        self.intake.intake_tote()
        return 'retreat_from_tote1'

    def retreat_from_tote1(self):
        self.drive.set_distance_goal(-15)
        return 'turn_away_from_tote1'

    def turn_away_from_tote1(self):
        self.drive.set_angle_goal(45)
        self.intake.outtake_tote()
        return 'move_toward_tote2'

    def move_toward_tote2(self):
        self.drive.set_distance_goal(84)
        return 'turn_to_tote2'

    def turn_to_tote2(self):
        self.intake.intake_tote()
        self.drive.set_angle_goal(0)
        return 'get_tote2'

    def get_tote2(self):
        self.drive.set_distance_goal(15)
        return 'retreat_from_tote2'

    def retreat_from_tote2(self):
        self.drive.set_distance_goal(-15)
        return 'turn_away_from_tote2'

    def turn_away_from_tote2(self):
        self.drive.set_angle_goal(45)
        self.intake.outtake_tote()
        return 'move_toward_tote3'

    def move_toward_tote3(self):
        self.drive.set_distance_goal(84)
        return 'turn_to_tote3'

    def turn_to_tote3(self):
        self.intake.intake_tote()
        self.drive.set_angle_goal(0)
        return 'get_tote3'

    def get_tote3(self):
        self.drive.set_distance_goal(15)
        return 'retreat_from_tote3'

    def retreat_from_tote3(self):
        self.drive.set_distance_goal(-15)
        return 'turn_toward_win'

    def turn_toward_win(self):
        self.drive.set_angle_goal(45 + 90)
        self.intake.pause()
        return 'go_to_win'

    def go_to_win(self):
        self.drive.set_distance_goal(100)
        return 'win'

    def win(self):
        self.drive.set_distance_goal(-5)
        self.elevator.drop_stack()
        self.intake.open()
        self.intake.outtake_tote()
        return 'secure_win'

    def secure_win(self):
        self.drive.set_distance_goal(-50)
        self.elevator.drop_stack()
        return 'stop'

    def stop(self):
        self.intake.close()
        self.intake.pause()
        self.elevator.drop_stack()
        self.drive.tank_drive(0, 0)
        return 'end'