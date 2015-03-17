from binascii import crc32
import struct
import socket
from twisted.internet.task import LoopingCall

# These are all unused in the code currently
"""Mode Bits
	RESET_BIT = 0x80;
	ESTOP_BIT = 0x40;
	ENABLED_BIT = 0x20;
	AUTONOMOUS_BIT = 0x10;
	FMS_ATTATCHED = 0x08;
	RESYNCH = 0x04;
	TEST_MODE_BIT = 0x02;
	CHECK_VERSIONS_BIT = 0x01;
"""
RESET_BIT = 0x80
FMS_ATTACHED = 0x08
RESYNCH = 0x04
CHECK_VERSIONS_BIT = 0x01

# All values found by trial-and-error. Feel free to update with better ones.
MODES = {
	'Teleoperated': 0x60,  # Not E-stop, Enabled 01100000
	'Autonomous': 0x70,  # Not E-stop, Enabled, Auto 01110000
	'Disabled': 0x40,  # Not E-stop
	'Test': 0x62,  # Not E-stop, Enabled, Test 01100010
	'Soft Reboot': 0x80,  # reset_encoder 1000 0000, doesn't need Not E-stop?
	'Emergency Stopped': 0x00  # ESTOP 0000 0000
}

roborio_port = 1110
blaze = 1150


class DSInfo(object):
	packetIndex = 0
	control = 0x40
	dsDigitalIn = 0xff
	teamID = 0x0000
	dsID_Alliance = 0x52
	dsID_Position = 0x31
	joystickAxes = [
		[0, 0, 0, 0, 0, 0],
		[0, 0, 0, 0, 0, 0],
		[0, 0, 0, 0, 0, 0],
		[0, 0, 0, 0, 0, 0]
	]

	joystickButtons = [0, 0, 0, 0]
	analogIn = [0, 0, 0, 0]


class DriverStation(object):
	update_interval = 0.02 * 1000
	connected = False
	missed_packets = 0

	client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	server.bind(('', blaze))
	server.setblocking(0)

	ds_info = DSInfo()
	robotData = None
	freeMemory = 0

	def __init__(self):
		self.send_timer = LoopingCall(self.send)
		self.find_timer = LoopingCall(self.send)
		self.receive_timer = LoopingCall(self.recieve)

	def start(self, team_id=0, alliance=None, position=None):
		self.set_team(team_id)
		self.alliance = alliance
		self.position = position

	def set_team(self, team_id):
		valid = (isinstance(team_id, int) and (1 < team_id < 9999))

		if valid:
			self.ip = "roborio-%s.local" % team_id
			self.ds_info.teamID = team_id
			if not self.connected and not self.find_timer.running:  # start if not connected
				print("Connecting to %s" % self.ip)
				self.wait_for_connection()
		else:
			self.disconnect()

	def wait_for_connection(self):
		print("Waiting for connection..")
		# Send packets at a slow rate to see if we get a response
		self.find_timer.start(self.update_interval * 50)

		# Start receiving packets from robot
		self.listen()

	def connect(self):
		self.send_timer.start(self.update_interval)

	def disconnect(self):
		self.send_timer.stop()

	def send(self):
		self.ds_info.packetIndex += 1
		packet = [0x00] * 1024  # empty

		packet[0:2] = struct.pack(">H", self.ds_info.packetIndex)

		packet[2] = struct.pack("B", self.ds_info.control)

		packet[3] = struct.pack("B", self.ds_info.dsDigitalIn)
		packet[4:6] = struct.pack(">H", self.ds_info.teamID)

		packet[6] = struct.pack("B", self.ds_info.dsID_Alliance)
		packet[7] = struct.pack("B", self.ds_info.dsID_Position)

		for i in range(4):
			for j in range(6):
				packet[(8 + j) + (i * 8)] = struct.pack("B", self.ds_info.joystickAxes[i][j])
			packet[14 + (i * 8): 16 + (i * 8)] = struct.pack(">H", self.ds_info.joystickButtons[i])

		packet[40:42] = struct.pack(">H", self.ds_info.analogIn[0])
		packet[42:44] = struct.pack(">H", self.ds_info.analogIn[1])
		packet[44:46] = struct.pack(">H", self.ds_info.analogIn[2])
		packet[46:48] = struct.pack(">H", self.ds_info.analogIn[3])

		c = bytearray([c if isinstance(c, int) else ord(c) for c in packet])
		packet[-4:] = struct.pack("I", crc32(c))
		c = bytearray([c if isinstance(c, int) else ord(c) for c in packet])

		try:
			print(c)
			self.client.sendto(c, (self.ip, roborio_port))
		except socket.gaierror:
			print("failed to send!")

		self.disconnectCheck()

	def listen(self):
		print("listening..")
		self.receive_timer.start(0.1)

	def recieve(self):
		print("t")
		try:
			data = self.server.recv(1024)
			if self.find_timer.running:
				self.find_timer.stop()
				self.connect()

			self.connected = True
			print(data)
			self.missed_packets = 0
		except socket.error as e:
			print(e)

	def disconnectCheck(self):
		if self.missed_packets > 10:
			self.disconnect()
			self.missed_packets = 0
			self.wait_for_connection()

	def enable(self, mode):
		mode_bits = MODES[mode]
		if mode_bits is None:
			print("Error!!!")
			return
		self.ds_info.control = mode_bits

	def disable(self):
		self.ds_info.control = MODES["Disabled"]

	def estop(self):
		self.ds_info.control = MODES["Emergency Stopped"]

	def reboot(self):
		self.ds_info.control = MODES["Soft Reboot"]

if __name__ == "__main__":
	ds = DriverStation()
	ds.start(team_id=865)
	ds.enable('Teleoperated')
	while True:
		pass