from panda3d.core import NodePath, Vec3
import numpy as np

class RadarActor(NodePath):
    def __init__(self, model_path, name, reflector_points=None):
        super().__init__(name)
        # Load model and attach to this NodePath
        self.model = loader.loadModel(model_path)
        self.model.reparentTo(self)

        # Reflector points in *model space*
        self.reflector_points = reflector_points or []

    def set_reflector_points(self, points):
        """Set local-space reflector points (list of Vec3 or tuples)."""
        self.reflector_points = [Vec3(*p) for p in points]

    def get_reflector_points_world(self):
        """Convert local-space reflector points to world-space positions."""
        world_points = []
        for p in self.reflector_points:
            world_points.append(self.get_mat(render).xform_point(p))
        return world_points

    def move(self, dx, dy, dz):
        """Simple movement for testing."""
        self.setPos(self, dx, dy, dz)

    def rotate(self, h, p, r):
        """Rotate the actor."""
        self.setHpr(self, h, p, r)

