# radar/input.py
from panda3d.core import Vec2
import time

class MouseHandler:
    def __init__(self, base, camera_controller=None):
        self.base = base
        self.camera_controller = camera_controller

        # State
        self.is_dragging = False
        self.mouse_down_pos = None
        self.mouse_down_time = 0

        # Sensitivity thresholds
        self.drag_threshold = 0.02
        self.drag_time_threshold = 0.05

        # Register mouse events
        base.accept("mouse1", self.on_mouse_down)
        base.accept("mouse1-up", self.on_mouse_up)
        base.accept("wheel_up", self.on_scroll_up)
        base.accept("wheel_down", self.on_scroll_down)

        # Track movement every frame
        base.taskMgr.add(self.track_mouse_task, "TrackMouseTask")

    def on_mouse_down(self):
        if not self.base.mouseWatcherNode.hasMouse():
            return
        self.mouse_down_pos = Vec2(
            self.base.mouseWatcherNode.getMouseX(),
            self.base.mouseWatcherNode.getMouseY()
        )
        self.mouse_down_time = time.time()
        self.is_dragging = False

    def on_mouse_up(self):
        if not self.base.mouseWatcherNode.hasMouse():
            return
        up_pos = Vec2(
            self.base.mouseWatcherNode.getMouseX(),
            self.base.mouseWatcherNode.getMouseY()
        )
        if not self.is_dragging:
            print(f"[Mouse] Click at {up_pos}")
        else:
            print(f"[Mouse] Drag end at {up_pos}")
        self.is_dragging = False

    def on_scroll_up(self):
        print("[Mouse] Scrolled up")
        if self.camera_controller:
            self.camera_controller.zoom(-1)

    def on_scroll_down(self):
        print("[Mouse] Scrolled down")
        if self.camera_controller:
            self.camera_controller.zoom(1)

    def track_mouse_task(self, task):
        if self.base.mouseWatcherNode.hasMouse() and self.mouse_down_pos is not None:
            current_pos = Vec2(
                self.base.mouseWatcherNode.getMouseX(),
                self.base.mouseWatcherNode.getMouseY()
            )
            dist = (current_pos - self.mouse_down_pos).length()

            if not self.is_dragging and dist > self.drag_threshold and \
               time.time() - self.mouse_down_time > self.drag_time_threshold:
                self.is_dragging = True
                print("[Mouse] Drag started")

            if self.is_dragging and self.camera_controller:
                dx = current_pos.x - self.mouse_down_pos.x
                dy = current_pos.y - self.mouse_down_pos.y
                self.camera_controller.orbit(dx, dy)
                self.mouse_down_pos = current_pos  # update reference

        return task.cont
