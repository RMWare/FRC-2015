import logging
from tkinter import *

from PIL import ImageTk, Image
import math
from networktables import NetworkTable


# To see messages from networktables, you must setup logging

logging.basicConfig(level=logging.DEBUG)


def scale(img, width):
	width_percent = (width / float(img.size[0]))
	height = int((float(img.size[1]) * float(width_percent)))
	img = img.resize((width, height), Image.ANTIALIAS)
	return img


size = 150


class Images():
	def __init__(self):
		self.tote_on = ImageTk.PhotoImage(scale(Image.open("tote_on.png"), size))
		self.tote_off = ImageTk.PhotoImage(scale(Image.open("tote_off.png"), size))
		self.can_on = ImageTk.PhotoImage(scale(Image.open("can_on.png"), size))
		self.can_off = ImageTk.PhotoImage(scale(Image.open("can_off.png"), size))


class App(Tk):
	def __init__(self):
		super().__init__()
		NetworkTable.getTable("Elevator").addTableListener(self.update_elevator)
		self.robot = NetworkTable.getTable('Tachyon')
		self.images = Images()
		self.countdown_label = Label(self, text="")
		self.countdown_label.grid(row=0, column=1)
		self.window_width_label = Label(self, text="                                             ")
		self.window_width_label.grid(row=1, column=1)
		self.can = Label(self, image=self.images.can_off)
		self.can.grid(row=0)
		self.totes = []
		for i in range(6):
			self.totes.append(Label(self, image=self.images.tote_off))
			self.totes[i].grid(row=i + 1)
		self.minutes = 0
		self.seconds = 0
		self.t = 150

		self.update()

	def update(self):
		# ##################### counter ######################
		t = self.robot.getNumber('match_time', 0.0)
		# t = self.t

		self.t -= 0.01

		minutes = t / 60
		s = round(t % 60)
		ms = round((t - math.floor(t)) * 100)

		if s < 10:
			seconds = "0" + str(s)
		else:
			seconds = str(s)

		if ms < 10:
			milliseconds = "0" + str(ms)
		else:
			milliseconds = str(ms)

		if t <= 1:
			self.countdown_label.configure(text="Time's up!", fg="black", font=("Britannic Bold", 20))
		else:
			self.countdown_label.configure(text="%d:" % minutes + seconds + ":" + milliseconds, fg="green" if t > 20 else "red", font=("Britannic Bold", 20))

		self.after(10, self.update)

	def update_elevator(self, source, key, value, is_new):
		# ##################### elevator ######################
		if key == 'has_bin':
			self.can.configure(image=self.images.can_on if value else self.images.can_off)
		elif key == '_tote_count':
			for index in range(int(value)):
				self.totes[4 - index].configure(image=self.images.tote_on if value else self.images.tote_off)
		elif key == 'has_game_piece':
			self.totes[-1].configure(image=self.images.tote_on if value else self.images.tote_off)


if __name__ == "__main__":
	NetworkTable.setIPAddress("roborio-865.local")
	NetworkTable.setClientMode()
	NetworkTable.initialize()
	app = App()
	app.mainloop()


