import functools
import inspect
from robotpy_ext.autonomous.stateful_autonomous import __get_state_serial


def distance_state(next_state=None, first=False):
	def real_dec(f=None):
		return conditional_state(f, done_func="drive.at_distance_goal", periodic_func="drive.drive_distance", next_state=next_state, first=first)
	return real_dec


def angle_state(next_state=None, first=False):
	def real_dec(f=None):
		return conditional_state(f, done_func="drive.at_angle_goal", periodic_func="drive.turn_angle", next_state=next_state, first=first)
	return real_dec


def conditional_state(f=None, done_func=None, periodic_func=None, next_state=None, first=False):
	if f is None:
		raise ValueError("conditional_state functions must specify a wrapped function")
	if done_func is None:
		raise ValueError("conditional_state functions must specify a done function")

	wrapper = _create_wrapper(f, first, done_func, periodic_func)

	wrapper.next_state = next_state

	return wrapper


def _create_wrapper(setup_func, first, done_func, periodic_func):
	@functools.wraps(setup_func)
	def wrapper(*args, **kwargs):
		if wrapper.first_run:
			setup_func(*args, **kwargs)
			wrapper.first_run = False
		if eval("self." + done_func + "()"):
			wrapper.expires = 0
			return
		eval("self." + periodic_func + "()")

	# store state variables here
	wrapper.name = setup_func.__name__
	wrapper.description = setup_func.__doc__
	wrapper.ran = False
	wrapper.first_run = True
	wrapper.first = first
	wrapper.expires = 0xffffffff
	wrapper.serial = __get_state_serial()

	# inspect the args, provide a correct call implementation
	args, varargs, keywords, _ = inspect.getargspec(setup_func)

	if keywords is not None:
		raise ValueError("Cannot use keyword arguments for function %s" % wrapper.name)

	if varargs is not None:
		raise ValueError("Cannot use *args arguments for function %s" % wrapper.name)

	if args[0] != 'self':
		raise ValueError("First argument must be 'self'")

	# TODO: there has to be a better way to do this. oh well. it only runs once.

	if len(args) > 4:
		raise ValueError("Too many parameters for %s" % wrapper.name)

	wrapper_creator = 'w = lambda self, tm, state_tm, initial_call: f(%s)' % ','.join(args)

	for arg in ['self', 'tm', 'state_tm', 'initial_call']:
		try:
			args.remove(arg)
		except ValueError:
			pass

	if len(args) != 0:
		raise ValueError("Invalid parameter names in %s: %s" % (wrapper.name, ','.join(args)))

	varlist = {'f': setup_func}
	exec(wrapper_creator, varlist, varlist)

	wrapper.run = varlist['w']
	return wrapper