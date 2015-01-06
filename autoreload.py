#!/usr/bin/env python3

import sys
import os
import time
import threading
import subprocess
import fnmatch

POLL_RATE = 1  # polling rate in seconds
TRIGGER_REGEX = "[!.]*.py"  # modification of any file that matches this regex will trigger reloads
SRC_LOCATION = "/home/admin/code/src"  # directory to monitor for reloads
PROGRAM = "main.py"  # robotpy entry point file


class AutoReload(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self._process = None
		self.files = self.get_files()
		self.restart_program()  # gotta start it once!

	def run(self):
		while True:
			time.sleep(POLL_RATE)
			if self.files_changed():
				print("Files changed! Reloading code...")
				self.restart_program()

	@staticmethod
	def get_files():
		file_list = []
		for root, dirs, files in os.walk(SRC_LOCATION):
			for filename in fnmatch.filter(files, TRIGGER_REGEX):
				full_filename = os.path.join(root, filename)
				file_list.append(full_filename)
		file_list = list(map(lambda f: (f, os.stat(f).st_mtime), file_list))  # & modification times!
		return file_list

	# def files_changed(self):
	# 	new_files = self.get_files()
	# 	if new_files != self.files:
	# 		self.files = new_files
	# 		return True
	# 	return False

	def files_changed(self):
		new_files = self.get_files()
		if self.files == new_files:
			return False
		else:
			self.files = new_files
			return True

	def restart_program(self):
		if self._process is not None and self._process.poll() is None:
			self._process.kill()
			self._process.wait()
		self._process = subprocess.Popen([sys.executable, os.path.join(SRC_LOCATION, PROGRAM), 'run'])


if __name__ == "__main__":
	AutoReload().start()
