import logging

log = logging.getLogger('robot')
from components import drive, intake, pneumatics, elevator  # These must be loaded after our hardware is.

class TachyAutonomous(object):

    #Inititalizes variables and sets what the first and last state is
    def __init__(self, components, first_state, end_state):
        self.drive = components.get('drive')
        self.pneumatics = components.get('pneumatics')
        self.intake = components.get('intake')
        self.elevator = components.get('elevator')
        self.first_iter = True
        self.state = None
        self.next_state = None
        self.first_state = first_state
        self.end_state = end_state
        self.goal = None
        log.info('initialized auton: %s' % self.__class__.__name__)

    def iterate(self):
        if self.state is None:
            self.state = self.first_state
        if self.goal or self.first_iter:
            if self.next_state is self.end_state:
                log.info('Auto has ended.')
                return
            else:
                if not self.first_iter:
                    self.state = self.next_state
                evalStr = 'self.%s()' % self.state
                log.info(evalStr) #Debugging
                self.next_state = eval(evalStr)
        self.first_iter = False
        log.info(self.state)
