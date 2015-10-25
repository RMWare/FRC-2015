import logging

log = logging.getLogger('robot')

class TachyAutonomous(object):

    #Inititalizes variables and sets what the first and last state is
    def __init__(self, components, first_state, end_state):
        self.components = components
        self.first_iter = False
        self.state = None
        self.next_state = None
        self.first_state = first_state
        self.end_state = end_state
        self.goal = None

    def iterate(self):
        if self.state is None:
            self.state = self.first_state
        if self.goal or self.first_iter:
            if self.next_state is self.end_state or self.next_state is None:
                log.info('Auto has ended.')
                return
            else:
                if not self.first_iter:
                    self.state = self.next_state
                evalStr = 'self.%s()' % self.state
                log.info(evalStr) #Debugging
                self.next_state = eval(evalStr)
        self.first_iter = False