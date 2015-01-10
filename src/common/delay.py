# Written by FRC1418

try:
    import wpilib
except ImportError:
    from pyfrc import wpilib


class PreciseDelay(object):
    """
        Used to synchronize a timing loop.

        Usage:

            delay = PreciseDelay(time_to_delay)

            while something:
                # do things here
                delay.wait()
    """
    
    def __init__(self, delay_period):
        """ :param delay_period: The amount of time to do a delay """
        
        self.timer = wpilib.Timer()        
        self.delay_period = delay_period
        
        self.timer.start()
        
    def wait(self):
        """ Waits until the delay period has passed """
        
        # we must *always* yield here, so other things can run
        wpilib.Timer.delay(0.001)
        
        while not self.timer.hasPeriodPassed(self.delay_period):
            wpilib.Timer.delay(0.001)
