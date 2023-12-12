### Motion planning helpers
class Node():
    """Simple node to keep track of 3-DOF pose and parentage"""
    def __init__(self, pose: tuple, parent: object=None):
        self._pose = pose
        self._parent = parent
    
    def set_parent(self, parent):
        self._parent = parent
    
    @property
    def pose(self):
        return self._pose
    
    @property
    def parent(self):
        return self._parent

### Simulation helpers
def move_arm(world, start, end):
    return