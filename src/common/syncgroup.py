class SyncGroup(object):
	"""
		This class acts as a proxy to run multiple robot components in parallel.
		For example, if you have 3 Talons you'd like to run at full speed
		s = SyncGroup(Talon, [0, 1, 2])
		s.set(1)
	"""

	def __init__(self, proxied_class, args_list):
		"""
		:param proxied_class: The class to instantiate
		:param args_list: A list of arguments per-instantiation ([1, 2] would do class(1) class(2), not class(1, 2).
		"""
		if len(args_list) == 0:
			raise ValueError("No arguments provided to instantiate the class with, therefore 0 instances created!")
		object.__setattr__(self, "_items",
		                   [proxied_class(*arg) if isinstance(arg, list) else proxied_class(arg) for arg in args_list]
		)

	def __getattribute__(self, item):
		ret = getattr(object.__getattribute__(self, "_items")[0], item)
		if hasattr(ret, "__call__"):
			return object.__getattribute__(self, "FunctionWrapper")(self, item)
		return ret

	class FunctionWrapper(object):
		def __init__(self, parent, func_name):
			self.parent = parent
			self.func_name = func_name

		def __call__(self, *args, **kwargs):
			ret = []
			for pwm in object.__getattribute__(self.parent, "_items"):
				item_func = getattr(pwm, self.func_name)
				ret.append(item_func(*args, **kwargs))
			if len(set(ret)) > 1:  # all values are not equal
				raise AssertionError("All items did not return the same value!!")
			return set(ret)  # Just return the last one, they should all be synced anyways.

	def __setattr__(self, key, value):
		for pwm in object.__getattribute__(self, "_items"):
			setattr(pwm, key, value)

	def __repr__(self):
		return "SyncGroup(%s)" % repr(object.__getattribute__(self, "_items"))