"""Minimal rospy stub for running mrim_planner outside ROS."""

_SENTINEL = object()

_params    = {}
_published = {}


def set_params(d):
    _params.clear()
    _params.update(d)


def get_published(topic):
    return _published.get(topic, [])


def clear_published():
    _published.clear()


def get_param(name, default=_SENTINEL):
    key = name.lstrip('~').lstrip('/')
    if key in _params:
        return _params[key]
    if default is not _SENTINEL:
        return default
    raise KeyError('rospy stub: parameter not found: {}'.format(name))


def init_node(name, anonymous=False):
    pass


def signal_shutdown(msg):
    print('[rospy stub] signal_shutdown: {}'.format(msg))


def spin():
    pass


def loginfo(msg, *args):
    print('[INFO] ' + (str(msg) % args if args else str(msg)))


def loginfo_once(msg, *args):
    loginfo(msg, *args)


def logwarn(msg, *args):
    print('[WARN] ' + (str(msg) % args if args else str(msg)))


def logerr(msg, *args):
    print('[ERROR] ' + (str(msg) % args if args else str(msg)))


class ROSInterruptException(Exception):
    pass


class Time:
    @staticmethod
    def now():
        return 0.0


class Rate:
    def __init__(self, hz):
        self.hz = hz

    def sleep(self):
        pass


class Publisher:
    def __init__(self, topic, msg_type=None, queue_size=1, latch=False):
        self.topic = topic

    def publish(self, msg):
        _published.setdefault(self.topic, []).append(msg)


class Subscriber:
    def __init__(self, *args, **kwargs):
        pass
