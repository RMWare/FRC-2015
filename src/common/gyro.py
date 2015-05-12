from _ctypes import POINTER
import ctypes
from wpilib import SPI, Timer
from wpilib.robotbase import logger

READ_COMMAND = 0x20
DATA_SIZE = 4
PARITY_BIT = 1
FIRST_BYTE_DATA = 0x3
THIRD_BYTE_DATA = 0xFC

WARM_UP_PERIOD = 5.0     # seconds
CALIBRATE_PERIOD = 15.0  # seconds


def bits(val):
	n = 0
	while val:
		val &= val - 1
		n += 1
	return n


class ADXRS453Z(object):
	def __init__(self, port):
		spi = SPI(port)
		spi.setClockRate(4000000)  # 4 MHz (rRIO max, gyro can go high)
		spi.setClockActiveHigh()
		spi.setChipSelectActiveLow()
		spi.setMSBFirst()
		self._spi = spi

		self._command = [0x00] * DATA_SIZE
		self._data = [0x00] * DATA_SIZE

		self._command[0] = READ_COMMAND

		self._accumulated_angle = 0.0
		self._current_rate = 0.0
		self._accumulated_offset = 0.0
		self._rate_offset = 0.0

		self._last_time = 0
		self._current_time = 0
		self._calibration_timer = Timer()
		self._calibration_timer.start()
		self._update_timer = Timer()
		self._update_timer.start()
		#
		# self._update_thread = Thread(self.update, daemon=True)
		# self._update_thread.start()
		self.update()

	def update(self):
		# Check parity
		num_bits = sum([bits(c) for c in self._command])
		if num_bits % 2 == 0:
			self._command[3] |= PARITY_BIT

		self._data = self._spi.transaction(self._command)

		if self._calibration_timer.get() < WARM_UP_PERIOD:
			self._last_time = self._current_time = self._update_timer.get()
			return
		elif self._calibration_timer.get() < CALIBRATE_PERIOD:
			self.calibrate()
		else:
			self.update_data()

	def update_data(self):
		sensor_data = self._assemble_sensor_data(self._data)
		rate = sensor_data / 80.0
		self._current_rate = rate
		self._current_rate -= self._rate_offset

		self._current_time = self._update_timer.get()
		dt = self._current_time - self._last_time

		self._accumulated_offset += rate * dt
		self._accumulated_angle += self._current_rate * dt
		self._last_time = self._current_time

	def calibrate(self):
		sensor_data = self._assemble_sensor_data(self._data)
		rate = sensor_data / 80.0
		self._current_rate = rate
		self._current_rate -= self._rate_offset

		self._current_time = self._update_timer.get()
		dt = self._current_time - self._last_time

		self._accumulated_offset += rate * dt
		self._rate_offset = self._accumulated_offset / (self._calibration_timer.get() - WARM_UP_PERIOD)
		self._last_time = self._current_time

	@staticmethod
	def _assemble_sensor_data(data):
		#cast to short to make space for shifts
		#the 16 bits from the gyro are a 2's complement short
		#so we just cast it too a C++ short
		#the data is split across the output like this (MSB first): (D = data bit, X = not data)
		# X X X X X X D D | D D D D D D D D | D D D D D D X X | X X X X X X X X X
		return (data[0] & FIRST_BYTE_DATA) << 14 | (data[1]) << 6 | (data[2] & THIRD_BYTE_DATA) >> 2

	def getRate(self):
		return self._current_rate

	def getAngle(self):
		return self._accumulated_angle

	def getOffset(self):
		return self._accumulated_offset

	def reset(self):
		self._data = [0x00] * 4
		self._current_rate = 0
		self._accumulated_angle = 0
		self._rate_offset = 0
		self._accumulated_offset = 0

		self._calibration_timer.reset()
		self._update_timer.reset()