# radar/scene.py
from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import CardMaker, TextureStage, LineSegs, NodePath, Filename
from .actor import RadarActor

class RadarSim(ShowBase):
    def __init__(self):
        super().__init__()

        self.disableMouse()
        self.camera.setPos(0, -25, 15)
        self.camera.lookAt(0, 0, 0)

        self.objects = []

        # Setup scene
        self.draw_ground()
        self.draw_axes()
        self.create_objects()

        # Camera rotation task
        self.taskMgr.add(self.spin_camera_task, "SpinCameraTask")

    def draw_ground(self):
        cm = CardMaker('ground')
        cm.setFrame(-100, 100, -100, 100)
        ground = self.render.attachNewNode(cm.generate())
        ground.setHpr(0, -90, 0)

        tex_path = Filename.from_os_specific("assets/textures/grass.png")
        grass_tex = self.loader.loadTexture(tex_path)
        ts = TextureStage('ts')
        ground.setTexture(ts, grass_tex)
        ground.setTexScale(ts, 50, 50)

    def draw_axes(self):
        lines = LineSegs()
        lines.setThickness(3)
        axis_len = 100

        # X-axis (red)
        lines.setColor(1, 0, 0, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(axis_len, 0, 0)

        # Y-axis (green)
        lines.setColor(0, 1, 0, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(0, axis_len, 0)

        # Z-axis (blue)
        lines.setColor(0, 0, 1, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(0, 0, axis_len)

        axis_np = NodePath(lines.create())
        axis_np.reparentTo(self.render)

    def create_objects(self):
        # Create a cube actor
        cube = RadarActor("models/box", "cube1")
        cube.reparentTo(self.render)
        cube.setPos(0, 0, 1)

        # Set up reflector points (8 corners of the cube)
        cube_points = [
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5)
        ]
        cube.set_reflector_points(cube_points)

        self.objects.append(cube)

        print("Cube reflector points (world):")
        for p in cube.get_reflector_points_world():
            print(p)

    def spin_camera_task(self, task):
        angle_deg = task.time * 6.0
        angle_rad = angle_deg * (pi / 180.0)
        self.camera.setPos(20 * sin(angle_rad), -20 * cos(angle_rad), 10)
        self.camera.lookAt(0, 0, 0)
        return Task.cont
