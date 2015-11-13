from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
## noinspect on PyUnresolvedReferences
# Because python is dynamic and can do crazy crap, some IDEs complain that the subsystems
# aren't accessible in this object. They are, and the above comment fixes it for IntelliJ IDEA


class ThreeTote(StatefulAutonomous):
    MODE_NAME = 'Three totes in auto zone'
    DEFAULT = True

    spin_direction = -1
    at_goal_state = ''

    def on_iteration(self, tm):
        self.elevator.auton()
        self.drive.auto_drive()
        super(ThreeTote, self).on_iteration(tm)

    @state()
    def drive_distance(self):
        if not self.drive.driving_distance:
            self.next_state(self.at_goal_state)

    @state()
    def drive_angle(self):
        if not self.drive.driving_angle:
            self.next_state(self.at_goal_state)


    @timed_state(first=True, duration=0.5, next_state='turn_away_from_tote1')
    def wait_for_elevator(self):
        self.intake.spin(1)
        self.intake.close()

    """
    @state()
    def get_first_tote(self):
        self.drive.set_distance_goal(15)
        self.intake.spin(1)
        self.at_goal_state = 'retreat_from_tote1'
        self.next_state('drive_distance')


    @state()
    def retreat_from_tote1(self):
        self.drive.set_distance_goal(-15)
        self.at_goal_state = 'turn_away_from_tote1'
        self.next_state('drive_distance')
    """

    @state()
    def turn_away_from_tote1(self):
        self.drive.set_angle_goal(45)
        self.at_goal_state = 'move_toward_tote2'
        self.next_state('drive_angle')

    @state()
    def move_toward_tote2(self):
        self.drive.set_distance_goal(45)
        # self.intake.spin(-1)
        self.at_goal_state = 'turn_to_tote2'
        self.next_state('drive_distance')

    @state()
    def turn_to_tote2(self):
        self.drive.set_angle_goal(2)
        self.at_goal_state = 'get_tote2'
        self.next_state('drive_angle')

    @state()
    def get_tote2(self):
        self.drive.set_distance_goal(15)
        self.at_goal_state = 'retreat_from_tote2'
        self.next_state('drive_distance')

    @state()
    def retreat_from_tote2(self):
        self.drive.set_distance_goal(-15)
        self.at_goal_state = 'turn_away_from_tote2'
        self.next_state('drive_distance')

    @state()
    def turn_away_from_tote2(self):
        self.drive.set_angle_goal(45)
        # self.intake.spin(-1)
        self.at_goal_state = 'move_toward_tote3'
        self.next_state('drive_angle')

    @state()
    def move_toward_tote3(self): # TODO
        self.drive.set_distance_goal(45)
        self.at_goal_state = 'turn_to_tote3'
        self.next_state('drive_distance')

    @state()
    def turn_to_tote3(self):
        # self.intake.intake(1)
        self.drive.set_angle_goal(2)
        self.at_goal_state = 'get_tote3'
        self.next_state('drive_angle')

    @state()
    def get_tote3(self):
        self.drive.set_distance_goal(15)
        self.at_goal_state = 'retreat_from_tote3'
        self.next_state('drive_distance')

    @state()
    def retreat_from_tote3(self):
        self.drive.set_distance_goal(-15)
        self.at_goal_state = 'turn_toward_win'
        self.next_state('drive_distance')

    @state()
    def turn_toward_win(self):
        self.drive.set_angle_goal(120)
        self.intake.stop()
        self.at_goal_state = 'go_to_win'
        self.next_state('drive_angle')

    @state()
    def go_to_win(self):
        self.drive.set_distance_goal(15)
        self.at_goal_state = 'win'
        self.next_state('drive_distance')

    @state()
    def win(self):
        self.drive.set_distance_goal(-5)
        self.elevator.drop_stack()
        self.intake.open()
        self.intake.spin(-1)
        self.at_goal_state = 'secure_win'
        self.next_state('drive_distance')

    @state()
    def secure_win(self):
        self.drive.set_distance_goal(-50)
        self.elevator.drop_stack()
        self.at_goal_state = 'stop'
        self.next_state('drive_distance')

    @state()
    def stop(self):
        self.intake.close()
        self.intake.spin(0)
        self.elevator.drop_stack()
        self.drive.tank_drive(0, 0)