class Component(object):
	def __init__(self):
		self.enabled = True

	def update(self):
		raise NotImplementedError("Component must have an update method")

	def fail(self):
		raise NotImplementedError("Component must have a fail method")
