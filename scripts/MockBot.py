
import struct
import time

class MockBot:
	def __init__(self):
		self.encoderleft = 0
		self.encoderright = 0
		self.updateTime = 0

	def motors(self, left, right):
		self.speedleft = left
		self.speedright = right

	def read_encoders(self):
		return struct.unpack('hh', bytes(bytearray([int(self.encoderleft), int(self.encoderright)])))
