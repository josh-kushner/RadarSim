# radar/camera.py
from math import sin, cos, pi

class CameraController:
    def __init__(self, base):
        self.base = base
        self.distance = 25
        self.angle_x = 0
        self.angle_y = 20  # vertical tilt

        self.update_camera()

    def orbit(self, dx, dy):
        # Adjust angles based on drag deltas
        self.angle_x += dx * 200  # sensitivity
        self.angle_y = max(-80, min(80, self.angle_y - dy * 200))
        self.update_camera()

    def zoom(self, direction):
        # Zoom in/out
        self.distance = max(5, min(100, self.distance + direction * 2))
        self.update_camera()

    def update_camera(self):
        rad_x = self.angle_x * pi / 180
        rad_y = self.angle_y * pi / 180
        x = self.distance * sin(rad_x)
        y = -self.distance * cos(rad_x)
        z = self.distance * sin(rad_y / 2)
        self.base.camera.setPos(x, y, z)
        self.base.camera.lookAt(0, 0, 0)
