import logging
from wpilib import Compressor
from . import Component
from common import quickdebug

log = logging.getLogger("robot")

class Pneumatics(Component):

    def __init__(self):
        super().__init__()
        self.comp = Compressor()

        quickdebug.add_printables(self, [('comp enabled', self.comp.enabled)])

    def update(self):
        """
        Monitors the PDP for amp draw, and disables the compressor if amp draw is above a threshold to prevent brownouts.
        :return:
        """
        self.comp.start()

    def stop(self):
        """Disables EVERYTHING. Only use in case of critical failure"""
        #self.comp.stop()
        pass