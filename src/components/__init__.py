class Component(object):
	def __init__(self):
		self.enabled = True

	def update(self):
		raise NotImplementedError("Component must have an 'update' method")

	def update_smartdashboard(self):
		pass

	def stop(self):
		raise NotImplementedError("Component must have a 'stop' method")