from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
import numpy as np
from direct.task import Task
from panda3d.core import (
    Vec3, CardMaker, TextureStage, LineSegs, NodePath, Filename,
    Point3, CollisionRay, CollisionNode, CollisionHandlerQueue, CollisionTraverser
)
from .actor import RadarActor
from .input import MouseHandler
from .camera import CameraController
import random


class RadarSim(ShowBase):
    def __init__(self):
        super().__init__()

        # Disable the default mouse-based camera control
        self.disableMouse()

        # Initialize camera + mouse helpers (from your own modules)
        self.camera_controller = CameraController(self)
        self.mouse_handler = MouseHandler(self, camera_controller=self.camera_controller)

        # Track targets in the scene
        self.targets = []

        self.LOS = []

        # ---------------------------
        # Collision and ray setup
        # ---------------------------
        self.ground_mask = 1
        self.picker = CollisionTraverser()
        self.pq = CollisionHandlerQueue()
        self.pickerNode = CollisionNode('mouseRay')
        self.pickerNP = self.camera.attachNewNode(self.pickerNode)
        self.pickerRay = CollisionRay()
        self.pickerNode.addSolid(self.pickerRay)
        self.picker.addCollider(self.pickerNP, self.pq)

        # ---------------------------
        # Scene setup
        # ---------------------------
        self.draw_ground()
        self.draw_axes()
        self.create_objects()

        # ---------------------------
        # Camera rotation controls
        # ---------------------------
        self.camera_angle = 0.0
        self.camera_distance = 25
        self.camera_height = 15
        self.rotation_speed = 60.0  # degrees per second
        self.taskMgr.add(self.update_camera_task, "UpdateCameraTask")

        # ---------------------------
        # Mouse input bindings
        # ---------------------------
        self.accept("mouse3", self.spawn_model_at_click)  # Right-click spawns
        self.accept("backspace", self.delete_targets)
        # Left-click hold handled in update_camera_task()

    # -------------------------
    # Scene drawing functions
    # -------------------------
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

        # Make ground pickable for ray casting
        ground.node().setIntoCollideMask(self.ground_mask)
        self.ground = ground

    def draw_axes(self):
        """Draw X (red), Y (green), and Z (blue) axes in the scene."""
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

    # -------------------------
    # Objects in the scene
    # -------------------------
    def create_objects(self):
        # Create radar actor in the center
        radar_path = Filename.from_os_specific("assets/models/StageSpotLight")
        radar = RadarActor(radar_path, "radar1")
        radar.reparentTo(self.render)
        radar.setHpr(-45, 0, 0)
        radar.setPos(0, 0, 1)

        # Set reflector points
        radar_points = [
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5)
        ]
        radar.set_reflector_points(radar_points)
        # self.targets.append(radar)

        print("radar reflector points (world):")
        for p in radar.get_reflector_points_world():
            print(p)

    def delete_targets(self):
        """Removes all spawned panda/radar objects from the scene."""
        for obj in self.targets:
            obj.removeNode()  # Remove the model from the scene graph
        for obj in self.LOS:
            obj.removeNode()
        self.targets.clear()  # Clear the list
        self.LOS.clear()
        print("All objects have been deleted.")

    def draw_sine_to_target(self, target, amplitude=0.5, wavelength=2.0, points=100):
        """
        Draw a sine wave from the radar to the target with a fixed wavelength.
        
        :param target: RadarActor or NodePath target
        :param amplitude: max displacement perpendicular to the line
        :param wavelength: length of one sine cycle in world units
        :param points: number of line segments to generate
        """
        start = Point3(0, 0, 1)  # radar position
        end = target.getPos(self.render)
        
        # Direction vector and distance
        direction = end - start
        length = direction.length()
        direction.normalize()
        
        # Pick a perpendicular vector for oscillation
        up = Vec3(1, 0, 0)
        perp = direction.cross(up).normalized()
        
        lines = LineSegs()
        lines.setThickness(2)
        lines.setColor(1, 1, 0, 1)  # yellow sine wave

        for i in range(points + 1):
            # Actual distance along the line
            dist = (i / points) * length
            point = start + direction * dist
            sine_offset = perp * (amplitude * sin(2 * pi * dist / wavelength))
            lines.drawTo(point + sine_offset)
        wave_np = self.render.attachNewNode(lines.create())
        self.LOS.append(wave_np)

    # -------------------------
    # Camera control
    # -------------------------
    def update_camera_task(self, task):
        """Rotate the camera while the left mouse button is held, 
        restricted to positive X, Y, Z coordinates."""
        dt = globalClock.getDt()

        if self.mouseWatcherNode.is_button_down('mouse1'):  # Left mouse held
            self.camera_angle += self.rotation_speed * dt

        angle_rad = self.camera_angle * (pi / 180.0)

        # Compute camera position (always positive X and Y)
        x = -1*abs(self.camera_distance * sin(angle_rad))
        y = -1*abs(self.camera_distance * cos(angle_rad))
        z = 0.75*abs(self.camera_height)

        self.camera.setPos(x, y, z)
        self.camera.lookAt(0, 0, 5)
        return Task.cont


    # -------------------------
    # Spawning new models
    # -------------------------
    def spawn_model_at_click(self):
        """Spawns a new radar where the user right-clicks on the ground."""
        if not self.mouseWatcherNode.hasMouse():
            return

        mpos = self.mouseWatcherNode.getMouse()
        self.pickerRay.setFromLens(self.camNode, mpos.getX(), mpos.getY())
        self.picker.traverse(self.render)

        if self.pq.getNumEntries() > 0:
            self.pq.sortEntries()
            entry = self.pq.getEntry(0)
            hit_pos = entry.getSurfacePoint(self.render)

            target = RadarActor("models/panda", f"radar_{len(self.targets)}")
            target.reparentTo(self.render)
            target.setPos(hit_pos + Point3(0, 0, 0))
            target.setScale(0.15)
            target.setHpr(random.random()*360, 0, 0)
            self.draw_sine_to_target(target, amplitude=0.25, wavelength=5)
            self.targets.append(target)

            print(f"Spawned target at {hit_pos}")
