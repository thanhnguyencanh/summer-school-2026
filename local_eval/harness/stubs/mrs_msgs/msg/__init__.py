class _Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Reference:
    def __init__(self):
        self.position = _Position()
        self.heading  = 0.0


class _Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp    = 0.0


class TrajectoryReference:
    def __init__(self):
        self.fly_now     = False
        self.use_heading = True
        self.loop        = False
        self.header      = _Header()
        self.points      = []
