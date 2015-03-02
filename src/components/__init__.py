class Component(object):
	def __init__(self):
		self.enabled = True

	def update(self):
		raise NotImplementedError("Component must have an update method")

	def update_smartdashboard(self):
		pass

	def fail(self):
		raise NotImplementedError("Component must have a fail method")