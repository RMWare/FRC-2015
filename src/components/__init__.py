class Component(object):
	def __init__(self):
		self.enabled = True
		self.tunables = []

	def update(self):
		raise NotImplementedError("Component must have an 'update' method")

	def stop(self):
		raise NotImplementedError("Component must have a 'stop' method")