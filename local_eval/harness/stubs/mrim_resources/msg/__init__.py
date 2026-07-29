class _Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class InspectionPoint:
    def __init__(self):
        self.idx             = 0
        self.position        = _Position()
        self.inspect_heading = 0.0
        self.inspect_tilt    = 0.0
        self.inspectability  = []
        self.type            = ''


class InspectionProblem:
    def __init__(self):
        self.name                        = ''
        self.comment                     = ''
        self.robot_ids                   = []
        self.start_poses                 = []
        self.inspection_points           = []
        self.obstacle_points             = []
        self.safety_area                 = []
        self.min_height                  = 0.0
        self.max_height                  = 0.0
        self.model_name                  = ''
        self.mesh_path                   = ''
        self.number_of_robots            = 0
        self.number_of_inspection_points = 0
        self.number_of_obstacle_points   = 0
