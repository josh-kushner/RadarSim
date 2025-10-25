from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import CardMaker, TextureStage, Texture, LineSegs, NodePath

class RadarSim(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        # Add the spinCameraTask procedure to the task manager.
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

        # Draw grass ground
        cm = CardMaker('ground')
        cm.setFrame(-100, 100, -100, 100)
        ground = self.render.attachNewNode(cm.generate())
        ground.setHpr(0, -90, 0)  # rotate to lie flat
        grass_tex = self.loader.loadTexture("grass.png")  # replace with your texture
        ts = TextureStage('ts')
        ground.setTexture(ts, grass_tex)
        ground.setTexScale(ts, 50, 50)  # tile texture

    # Function to draw 3D axes (X = red, Y = green, Z = blue)
        thickness = 5
        length = 999
        lines = LineSegs()
        lines.setThickness(thickness)
        # X-axis (red)
        lines.setColor(1, 0, 0, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(length, 0, 0)
        # Y-axis (green)
        lines.setColor(0, 1, 0, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(0, length, 0)
        # Z-axis (blue)
        lines.setColor(0, 0, 1, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(0, 0, length)
        # Create a NodePath from the lines and attach it to render
        axis_node = lines.create()
        axis_np = NodePath(axis_node)
        axis_np.reparentTo(self.render)

    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

app = RadarSim()
app.run()
