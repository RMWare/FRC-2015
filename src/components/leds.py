from wpilib import SPI
from common.colors import Color
from components import Component


class LEDStrip(Component):
	def __init__(self):
		self.spi = SPI(0)
		self.spi.setClockRate(12000000)

		self.leds = 22
		self.lastIndex = self.leds - 1
		self.gamma = bytearray(256)
		self.buffer = [0 for x in range(self.leds + 1)]

		self.masterBrightness = 1.0

		for led in range(self.leds):
			self.buffer[led] = bytearray(3)

		for i in range(256):
			# Color calculations from
			# http://learn.adafruit.com/light-painting-with-raspberry-pi
			self.gamma[i] = 0x80 | int(
				pow(float(i) / 255.0, 2.5) * 127.0 + 0.5
			)
		self.fill_RGB(1, 0, 0)

	def set_brightness(self, bright):
		if bright > 1.0 or bright < 0.0:
			raise ValueError('Brightness must be between 0.0 and 1.0')
		self.masterBrightness = bright

	def fill(self, color, start=0, end=-1):
		if start < 0:
			start = 0
		if end < 0 or end > self.lastIndex:
			end = self.lastIndex
		for led in range(start, end + 1):
			self.__set_internal(led, color)

	def fill_RGB(self, r, g, b, start=0, end=-1):
		self.fill(Color(r, g, b), start, end)

	def __set_internal(self, pixel, color):
		if pixel < 0 or pixel > self.lastIndex:
			return

		self.buffer[pixel][1] = self.gamma[int(color.r * self.masterBrightness)]
		self.buffer[pixel][0] = self.gamma[int(color.g * self.masterBrightness)]
		self.buffer[pixel][2] = self.gamma[int(color.b * self.masterBrightness)]

	def update(self):
		for x in range(self.leds):
			self.spi.write(self.buffer[x])
		self.spi.write(bytearray(b'\x00\x00\x00'))  # zero fill the last to prevent stray colors at the end

	def stop(self):
		pass