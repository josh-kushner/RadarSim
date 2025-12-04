from panda3d.core import NodePath, Vec3
import numpy as np

class RadarActor(NodePath):
    def __init__(self, model_path, name):
        super().__init__(name)
        # Load model and attach to this NodePath
        self.model = loader.loadModel(model_path)
        self.model.reparentTo(self)