"""
ECE 239AS Implementation Final Project: RadarSim (FMCW Radar Simulation)
Written by Josh Kushner and Michael Zhou
"""

from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from direct.task import Task
from panda3d.core import (
    CardMaker, TextureStage, LineSegs, NodePath, Filename,
    Point3, CollisionRay, CollisionNode, CollisionHandlerQueue, CollisionTraverser,
    TextNode
)
from .actor import RadarActor
from .camera import CameraController
from .signals import FMCWDSP
import matplotlib
matplotlib.use("Agg")  # Avoids Tk error when generating plots
import matplotlib.pyplot as plt
import numpy as np

RADAR_POS = Point3(0, 0, 0) # Radar position (x, y, z)

class RadarSim(ShowBase): 
    def __init__(self):
        super().__init__()

        # Initialize camera controller
        self.disableMouse()
        self.camera_controller = CameraController(self)

        self.targets = [] # Target actor objects
        self.LOS = [] # Line of sight visualization lines
        
        self.c = 3e8 # speed of light
        
        # Arrays of radar parameters
        self.freq_arr = np.arange(100e9, 130e9, 10e9)
        self.bandwidth_arr = np.arange(2e9, 3.5e9, 0.5e9)
        self.chirp_arr = np.arange(40e-6, 55e-6, 5e-6)
        self.frame_arr = np.arange(40, 55, 5)
        self.freq_idx = 0
        self.bandwidth_idx = 0
        self.chirp_idx = 0
        self.frame_idx = 0
        self.velocity_idx = 0
        
        # Initialize default radar parameters
        self.starting_freq = self.freq_arr[self.freq_idx]
        self.bandwidth = self.bandwidth_arr[self.bandwidth_idx]
        self.chirp_time = self.chirp_arr[self.chirp_idx]
        self.frame_size = self.frame_arr[self.frame_idx]
        
        # Calculate derived values
        self.lam = self.c / (self.starting_freq + self.bandwidth/2) # wavelength
        self.max_vel = self.lam / (4 * self.chirp_time) # maximum non-ambiguous velocity
        self.velocity_arr = np.floor(np.linspace(0, self.max_vel, 5))
        self.velocity = self.velocity_arr[self.velocity_idx]
        
        self.sampling_freq = 2.1 * self.bandwidth # sampling frequency of ADC

        # Maximum Unambiguous Range
        # print(self.c * self.chirp_time * self.sampling_freq / (2 * self.bandwidth))

        # State Machine for app
        # states = ["SETUP", "RUN", "END"]
        self.state_idx = 0

        # Global Timer
        self.start_time = self.taskMgr.globalClock.getFrameTime()  # record start time
        self.cur_time = 0

        # Create text boxes
        x = 1.5 
        y0 = 0.90
        dy = 0.08
    
        self.space_text = OnscreenText(
            text=f"PRESS SPACE TO START",
            pos=(0, 0),
            align=TextNode.A_center,
            scale=0.05,
            fg=(1, 1, 1, 1)
        )

        self.freq_text = OnscreenText(
            text=f"[1] STARTING FREQ: {self.starting_freq / 1e9} GHz",
            pos=(x, y0),
            align=TextNode.A_right,
            scale=0.05,
            fg=(1, 1, 1, 1)
        )

        self.bandwidth_text = OnscreenText(
            text=f"[2] BANDWIDTH: {self.bandwidth / 1e9} GHz",
            pos=(x, y0 - dy),
            align=TextNode.A_right,
            scale=0.05,
            fg=(1, 1, 1, 1)
        )

        self.chirp_text = OnscreenText(
            text=f"[3] CHIRP TIME: {self.chirp_time * 1e6} us",
            pos=(x, y0 - 2*dy),
            align=TextNode.A_right,
            scale=0.05,
            fg=(1, 1, 1, 1)
        )

        self.frame_text = OnscreenText(
            text=f"[4] FRAME SIZE: {self.frame_size}",
            pos=(x, y0 - 3*dy),
            align=TextNode.A_right,
            scale=0.05,
            fg=(1, 1, 1, 1),
            wordwrap=0
        )

        self.velocity_text = OnscreenText(
            text=f"[5] VELOCITY: {self.velocity} m/s",
            pos=(x, y0 - 4*dy),
            align=TextNode.A_right,
            scale=0.05,
            fg=(1, 1, 1, 1)
        )

        self.lmb_text = OnscreenText(
            text=f"LEFT MOUSE: ROTATE CAMERA",
            pos=(-x, y0),
            align=TextNode.A_left,
            scale=0.05,
            fg=(1, 1, 1, 1),
            wordwrap=0
        )

        self.rmb_text = OnscreenText(
            text=f"RIGHT MOUSE: SPAWN TARGET",
            pos=(-x, y0 - dy),
            align=TextNode.A_left,
            scale=0.05,
            fg=(1, 1, 1, 1),
            wordwrap=0
        )

        self.backspace_text = OnscreenText(
            text=f"BACKSPACE: CLEAR TARGETS",
            pos=(-x, y0 - 2*dy),
            align=TextNode.A_left,
            scale=0.05,
            fg=(1, 1, 1, 1),
            wordwrap=0
        )

        self.timer_text = OnscreenText(
            text=f"{self.cur_time}",
            pos=(x/2, -y0),
            align=TextNode.A_center,
            scale=0.05,
            fg=(1, 1, 1, 1),
            wordwrap=0
        )

        self.end_text = OnscreenText(
            text=f"END SIMULATION",
            pos=(0, -0.3),
            align=TextNode.A_center,
            scale=0.05,
            fg=(1, 1, 1, 1),
            wordwrap=0
        )
        self.end_text.hide()
        
        # ---------------------------
        # Mouse picking setup for 3D world interaction
        # Converts 2D mouse clicks to 3D coordinates via ray casting
        # ---------------------------
        self.ground_mask = 1
        self.picker = CollisionTraverser()
        self.pq = CollisionHandlerQueue()
        self.pickerNode = CollisionNode('mouseRay')
        self.pickerNP = self.camera.attachNewNode(self.pickerNode)
        self.pickerRay = CollisionRay()
        self.pickerNode.addSolid(self.pickerRay)
        self.picker.addCollider(self.pickerNP, self.pq)

        # Spawn grass ground, 3D axes, and radar model
        self.draw_ground()
        self.draw_axes()
        self.create_radar()

        # FMCW Signal Processor initialization
        self.signal_processor = FMCWDSP(
            bandwidth=self.bandwidth,
            chirp_time=self.chirp_time,
            starting_freq=self.starting_freq,
            sampling_freq=self.sampling_freq,
            frame_size=self.frame_size,
            velocity=self.velocity,
            radar_pos=RADAR_POS
        )

        # Camera configuration
        self.camera_angle = 0.0
        self.camera_distance = 25
        self.camera_height = 15
        self.rotation_speed = 60.0  # degrees per second

        # Input controls
        self.accept("1", self.toggle_freq)
        self.accept("2", self.toggle_bandwidth)
        self.accept("3", self.toggle_chirp)
        self.accept("4", self.toggle_frame)
        self.accept("5", self.toggle_velocity)
        self.accept("space", self.toggle_state)
        self.accept("mouse3", self.spawnTarget)
        self.accept("backspace", self.delete_targets)

        # Task Manager
        # These functions are called each frame
        self.taskMgr.add(self.update_camera_task, "UpdateCameraTask")
        self.taskMgr.add(self.move_targets_task, "MoveTargetsTask")
        self.taskMgr.add(self.update_timer, "TimerTask")

    ############################################################################
    # Scene Setup Logistics                                                    #
    ############################################################################
    
    def draw_ground(self):
        """
        Duplicates grass image to draw a flat ground.
        """
        cm = CardMaker('ground')
        cm.setFrame(-100, 100, -100, 100)
        ground = self.render.attachNewNode(cm.generate())
        ground.setHpr(0, -90, 0)

        # Apply grass texture with tiling
        tex_path = Filename.from_os_specific("assets/textures/grass.png")
        grass_tex = self.loader.loadTexture(tex_path)
        ts = TextureStage('ts')
        ground.setTexture(ts, grass_tex)
        ground.setTexScale(ts, 50, 50)

        # Enable collision detection for ray-picking
        ground.node().setIntoCollideMask(self.ground_mask)
        self.ground = ground

    def draw_axes(self):
        """
        Draw 3D axes.
        X (red)
        Y (green)
        Z (blue)
        """
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

    def create_radar(self):
        """
        Spawns radar.
        """
        radar_path = Filename.from_os_specific("assets/models/StageSpotLight")
        radar = RadarActor(radar_path, "radar1")
        radar.reparentTo(self.render)
        radar.setHpr(-45, 0, 0)
        radar.setPos(RADAR_POS)

    def drawLOS(self, target):
        """
        Draw line-of-sight from radar to target.
        """
        start = RADAR_POS
        end = target.getPos(self.render)
        lines = LineSegs()
        lines.setThickness(2)
        lines.setColor(1, 1, 0, 1)  # Yellow line
        lines.moveTo(start)
        lines.drawTo(end)
        node = self.render.attachNewNode(lines.create())
        self.LOS.append(node)

    ############################################################################
    # Frame-to-Frame Tasks                                                     #
    ############################################################################

    def update_timer(self, task):
        """
        Update the global timer
        """
        if self.state_idx == 1:
            self.cur_time = self.taskMgr.globalClock.getFrameTime() - self.start_time
            self.timer_text.setText(f"{self.cur_time:.2f}")
        return task.cont

    def update_camera_task(self, task):
        """
        Update camera position each frame.
        Rotate the camera when left mouse is held.
        """
        dt = self.clock.getDt()

        # Rotate camera when left mouse button is held
        if self.mouseWatcherNode.is_button_down('mouse1'):
            self.camera_angle += self.rotation_speed * dt

        angle_rad = self.camera_angle * (pi / 180.0)

        # Calculate camera position on a circular path
        x = -1 * abs(self.camera_distance * sin(angle_rad))
        y = -1 * abs(self.camera_distance * cos(angle_rad))
        z = 0.75 * abs(self.camera_height)

        self.camera.setPos(x, y, z)
        self.camera.lookAt(0, 0, 5)
        return Task.cont

    def move_targets_task(self, task):
        """
        Moves spawned targets on the line-of-sight.
        """
        if self.state_idx == 1:
            dt = self.clock.getDt()

            for target in self.targets:
                # Move the target
                move_vec = target.dir * target.velocity * dt
                target.setPos(target.getPos() + move_vec)

                # Make it face where it's moving
                look_pos = target.getPos() + target.dir
                target.lookAt(look_pos)

                # ------- Range checking for back-and-forth motion -------
                start = target.start_pos
                end = target.end_pos
                cur = target.getPos()

                # Project the current position onto the motion vector
                path_vec = end - start
                path_len_sq = path_vec.lengthSquared()

                proj = (cur - start).dot(path_vec)

                # If before start or past end, reverse direction
                if proj < 0:
                    # Clamp to start
                    target.setPos(start)
                    # Flip direction
                    target.dir *= -1
                    # Swap endpoints
                    target.start_pos, target.end_pos = end, start

                elif proj > path_len_sq:
                    # Clamp to end
                    target.setPos(end)
                    # Flip direction
                    target.dir *= -1
                    # Swap endpoints
                    target.start_pos, target.end_pos = end, start
        return task.cont
    
    ############################################################################
    # User Preference Toggling                                                 #
    ############################################################################
    
    def toggle_state(self):
        """
        State Machine for simulation.
        """
        tx, rx = [], []
        if self.state_idx == 1:
            # base.graphicsEngine.renderFrame()
            if len(self.targets) != 0:
                tx, rx = self.start_chirp_frame()
        self.state_idx += 1
        self.state_idx = self.state_idx % 3
        if self.state_idx == 0: # SETUP
            self.timer_text.setText("0")
            self.space_text.setText("PRESS SPACE TO START")
            self.freq_text.show()
            self.bandwidth_text.show()
            self.chirp_text.show()
            self.velocity_text.show()
            self.lmb_text.show()
            self.rmb_text.show()
            self.backspace_text.show()
            self.frame_text.show()
            self.end_text.hide()
        elif self.state_idx == 1: # RUN
            self.space_text.setText("PRESS SPACE TO STOP")
            self.start_time = self.taskMgr.globalClock.getFrameTime()  # record start time
            self.cur_time = 0
            self.freq_text.hide()
            self.bandwidth_text.hide()
            self.chirp_text.hide()
            self.velocity_text.hide()
            self.lmb_text.show()
            self.rmb_text.hide()
            self.frame_text.hide()
            self.backspace_text.hide()
        elif self.state_idx == 2: # END
            self.end_text.show()
            self.space_text.setText("PRESS SPACE TO START OVER\n\nPLOTS SAVED IN CURRENT FOLDER")
            if len(self.targets) != 0:
                self.plot_range_resolution()
                self.plot_velocity_resolution()
                self.plot_chirp(tx, rx)

    def toggle_freq(self):
        """
        Toggle radar starting frequency.
        """
        if self.state_idx != 0: return
        self.freq_idx = (self.freq_idx + 1) % len(self.freq_arr)
        self.starting_freq = self.freq_arr[self.freq_idx]
        starting_freq = 0
        starting_freq = self.starting_freq / 1e9
        self.freq_text.setText(f"[1] STARTING FREQ: {starting_freq} GHz")
        self.signal_processor.setStartingFreq(self.starting_freq)

        self.lam = self.c / (self.starting_freq + self.bandwidth/2)
        self.max_vel = self.lam / (4 * self.chirp_time)
        self.velocity_arr = np.floor(np.linspace(0, self.max_vel, 5))
        self.velocity = self.velocity_arr[self.velocity_idx % len(self.velocity_arr)]
        self.velocity_text.setText(f"[5] VELOCITY: {self.velocity} m/s")

    def toggle_bandwidth(self):
        """
        Toggle radar bandwidth.
        """
        if self.state_idx != 0: return
        self.bandwidth_idx += 1
        self.bandwidth = self.bandwidth_arr[self.bandwidth_idx % len(self.bandwidth_arr)]
        bandwidth = self.bandwidth / 1e9
        self.bandwidth_text.setText(f"[2] BANDWIDTH: {bandwidth} GHz")
        self.signal_processor.setBandwidth(self.bandwidth)
        self.lam = self.c / (self.starting_freq + self.bandwidth/2)
        self.max_vel = self.lam / (4 * self.chirp_time)
        self.velocity_arr = np.floor(np.linspace(0, self.max_vel, 5))
        self.velocity = self.velocity_arr[self.velocity_idx % len(self.velocity_arr)]
        self.velocity_text.setText(f"[5] VELOCITY: {self.velocity} m/s")
    
    def toggle_chirp(self):
        """
        Toggle radar chirp time.
        """
        if self.state_idx != 0: return
        self.chirp_idx += 1
        self.chirp_time = self.chirp_arr[self.chirp_idx % len(self.chirp_arr)]
        chirp_time = 0
        chirp_time = self.chirp_time * 1e6
        self.chirp_text.setText(f"[3] CHIRP TIME: {chirp_time} us")
        self.signal_processor.setChirpTime(self.chirp_time)
        self.max_vel = self.lam / (4 * self.chirp_time)
        self.velocity_arr = np.floor(np.linspace(0, self.max_vel, 5))
        self.velocity = self.velocity_arr[self.velocity_idx % len(self.velocity_arr)]
        self.velocity_text.setText(f"[5] VELOCITY: {self.velocity} m/s")

    def toggle_frame(self):
        """
        Toggle radar frame size.
        """
        if self.state_idx != 0: return
        self.frame_idx += 1
        self.frame_size = self.frame_arr[self.frame_idx % len(self.frame_arr)]
        self.frame_text.setText(f"[4] FRAME SIZE: {self.frame_size}")
        self.signal_processor.setFrameSize(self.frame_size)

    def toggle_velocity(self):
        """
        Toggle velocity of target to spawn.
        """
        if self.state_idx != 0: return
        self.velocity_idx += 1
        self.velocity = self.velocity_arr[self.velocity_idx % len(self.velocity_arr)]
        self.velocity_text.setText(f"[5] VELOCITY: {self.velocity} m/s")
    
    ############################################################################
    # Target Manipulation                                                      #
    ############################################################################

    def delete_targets(self):
        """
        Remove all spawned targets and their line-of-sights.
        """
        if self.state_idx != 0: return
        for obj in self.targets:
            obj.removeNode()
        for obj in self.LOS:
            obj.removeNode()
        self.targets.clear()
        self.LOS.clear()

    def spawnTarget(self):
        """
        Spawns targets. Uses the mouse position and where the clicks 
        are registered to map to a corresponding coordinate in the 
        simulated 3D environment.
        """
        if self.state_idx != 0: return
        if not self.mouseWatcherNode.hasMouse(): return

        mpos = self.mouseWatcherNode.getMouse()
        self.pickerRay.setFromLens(self.camNode, mpos.getX(), mpos.getY())
        self.picker.traverse(self.render)

        if self.pq.getNumEntries() > 0:
            self.pq.sortEntries()
            entry = self.pq.getEntry(0)
            hit_pos = entry.getSurfacePoint(self.render)
            # Only allow targets to be spawned in front of radar
            if hit_pos.getX() < 0 or hit_pos.getY() < 0:
                return

            # Create target actor
            target = RadarActor("models/panda", f"panda_{len(self.targets)}")
            target.reparentTo(self.render)
            target.setPos(hit_pos)
            target.setScale(0.15)

            # ------------------------
            # Linear motion attributes
            # ------------------------
            target.start_pos = hit_pos
            target.dir = RADAR_POS - hit_pos
            target.dir.setZ(0)
            target.dir.normalize()

            target.end_pos = RADAR_POS
            target.velocity = self.velocity

            # Visualize and process
            self.drawLOS(target)
            self.targets.append(target)

    ############################################################################
    # Radar Actions                                                            #
    ############################################################################

    def start_chirp_frame(self):
        """
        Purpose:
            Generate FMCW chirps for all targets, stack fast-time signals along
            slow time, and estimate velocity from phase differences.
        Input:
            N/A
        Output:
            tx: the transmit signal from the transmitter
            rx: the receiver signal reflected from the target
        """

        f_center = self.starting_freq + self.bandwidth / 2
        sum_beat_lpf = None
        tx = []
        rx = []

        for i, target in enumerate(self.targets):
            print()
            print("---------------------------------------------------")
            # Interpolate target distances for all chirps (slow time axis)
            dists = self.interpolate_pos(target)
            beat_lpfs = []
            time_delays = []
            true_dist = 0
            fmcw_range = 0
        
            for dist in dists:
                results = self.signal_processor.generate_chirp(dist)
                if len(tx) == 0 or len(rx) == 0:
                    tx = results['tx']
                    rx = results['rx']
                beat_lpfs.append(results['beat_lpf'])
                time_delays.append(results['time_delay'])
                true_dist = results['true_range']
                fmcw_range = results['estimated_range']

            print(f"TRUE DISTANCE: {true_dist}, FMCW DISTANCE: {fmcw_range}")
        
            beat_lpfs = np.array(beat_lpfs)

            if sum_beat_lpf is None:
                sum_beat_lpf = beat_lpfs
            else:
                sum_beat_lpf = sum_beat_lpf + beat_lpfs
        
            # Compute phase differences for velocity estimation
            phase_diffs = []
            for i in range(1, len(time_delays)):
                delta_td = time_delays[i] - time_delays[i-1]
                phase_diff = -2 * np.pi * f_center * delta_td
                phase_diffs.append(phase_diff)
            if phase_diffs:
                avg_phase_diff = np.mean(phase_diffs)
                self.signal_processor.calculateVelocity(avg_phase_diff, target)
        
        if sum_beat_lpf is not None:
            # Generate Range FFT from combined signals
            self.generate_range_fft(sum_beat_lpf[0])

            # Generate Range-Doppler map
            self.generate_rdm(sum_beat_lpf)

        return tx, rx
    
    ############################################################################
    # Plotting                                                                 #
    ############################################################################
    
    def generate_range_fft(self, beat_lpf):
        """
        Purpose:
            Generate the range FFT plot for all targets combined
        Input:
            beat_lpf: an arbitrary chirp from the burst to analyze
        Output:
            N/A
        """
        N = len(beat_lpf)
        fft_vals = np.fft.fft(beat_lpf)
        freqs = np.fft.fftfreq(N, 1 / self.sampling_freq)

        # Only positive frequencies
        pos_mask = freqs > 0
        fft_vals_pos = fft_vals[pos_mask]
        freqs = freqs[pos_mask]
        ranges = (self.c * freqs * self.chirp_time) / (2 * self.bandwidth)

        fft_mag_db = 20 * np.log10(np.abs(fft_vals_pos) + 1e-12)
        fft_mag_db /= np.max(fft_mag_db)  # normalize for plotting

        # ----------------------------------------------
        # Helper: find top N peaks for labeling purposes
        # ----------------------------------------------
        def find_top_N_peaks(vals, ranges, N, threshold=None):
            # Apply threshold if specified
            if threshold is not None:
                valid_mask = vals >= threshold
                valid_indices = np.where(valid_mask)[0]
                
                if len(valid_indices) == 0:
                    # No peaks above threshold
                    return np.array([]), np.array([]), np.array([])
                
                # Only consider values above threshold
                valid_vals = vals[valid_indices]
                top_N = min(N, len(valid_vals))
                
                # Find top N among valid values
                top_relative_indices = np.argsort(valid_vals)[-top_N:][::-1]
                top_indices = valid_indices[top_relative_indices]
            else:
                # No threshold - original behavior
                top_N = min(N, len(vals))
                top_indices = np.argsort(vals)[-top_N:][::-1]

            return ranges[top_indices], vals[top_indices]

        # Get top N peaks
        top_ranges, top_vals = find_top_N_peaks(fft_mag_db, ranges, len(self.targets), threshold=None)

        # Plot
        plt.figure(figsize=(8,5))
        plt.plot(ranges, fft_mag_db, label="Range FFT")
        plt.scatter(top_ranges, top_vals, color='red', label='Top peaks')

        # Annotate top peaks (slightly to the right, 2 decimal places)
        for r, v in zip(top_ranges, top_vals):
            plt.text(r + 0.5, v + 0.02, f"{r:.2f}", color='red', ha='left', va='bottom', fontsize=8)

        # Plot true target ranges
        for i, target in enumerate(self.targets):
            true_dist = (target.getPos() - RADAR_POS).length()
            # Only add label for first line to avoid duplicates in legend
            label = 'True range' if i == 0 else None
            plt.axvline(true_dist, color='green', linestyle=':', linewidth=1.5, label=label)
            # Annotate true distance slightly to the right, 2 decimal places
            plt.text(true_dist + 0.5, 0.02, f"{true_dist:.2f}", color='green', ha='left', va='bottom', fontsize=8)

        plt.xlabel("Range (m)")
        plt.ylabel("Normalized Magnitude")
        plt.title(f"Range FFT (Range Res: {self.c / (2 * self.bandwidth):.2f} m)")
        plt.xlim([0, 150])
        plt.grid(True)
        plt.legend()
        plt.savefig('RangeFFT.png')
        plt.close()

        print("** Generated Range FFT Plot @ RangeFFT.png")

    def generate_rdm(self, stack_beat_lpfs):
        """
        Purpose:
            Generate Range-Doppler Map from stacked beat signals.
        Input:
            stack_beat_lpfs: an array of the beat signals "stacked" together
        Output:
            N/A
        """
        N = stack_beat_lpfs.shape[1]  # M = num chirps, N = num samples
        
        # Calculate range axis
        fs = self.sampling_freq
        fft_len = N * 8

        rmax = self.c * self.chirp_time * fs / (2 * self.bandwidth)
        ranges = np.linspace(-rmax / 2, rmax / 2, fft_len)
        
        rdm = np.fft.fftshift(np.abs(np.fft.fft2(stack_beat_lpfs.T))) / (N / 2)
        
        # Convert to dB
        rdm = 10 * np.log10(rdm + 1e-22)

        extent = [-self.max_vel, self.max_vel, ranges.min(), ranges.max()]        
        # Plot
        fig, ax = plt.subplots(figsize=(8, 8))
        range_doppler_plot = ax.imshow(
            rdm,
            aspect="auto",
            extent=extent,
            origin="lower",
            vmax=2,
            vmin=-25,
        )
        ax.set_ylim([0, 150])
        ax.set_title(f"Range Doppler Map (Vel Res: {(self.c / (self.starting_freq + self.bandwidth / 2))/(2 * self.frame_size * self.chirp_time):.2f} m/s)", fontsize=24)
        ax.set_xlabel("Velocity (m/s)", fontsize=22)
        ax.set_ylabel("Range (m)", fontsize=22)
        fig.colorbar(range_doppler_plot, label='dB')
        plt.tight_layout()
        plt.savefig("RangeDopplerMap.png", dpi=200)
        plt.close()

        print("** Generated Range-Doppler Map @ RangeDopplerMap.png")
        

    def interpolate_pos(self, target):
        """
        Purpose:
            Since the 3d environment cannot reasonably refresh at a rate of
            25000 FPS (to match our shortest chirp time of 40 us), we cannot
            simulate the distance across chirps at the granularity of microseconds,
            so we opted to generously estimate based on the current trajectory of
            motion.
        Input:
            target: the indicated target we are performing the interpolation on
        Output:
            dists: an array of interpolated distances for the current target in
                   the next N frames (where N is the desired frame_size)
        """
        # Initial distance of target (obtained from simulated 3D environment)
        dist = (target.getPos() - RADAR_POS).length()
        
        # Future distances of target (interpolated via physics)
        dists = [dist]
        for i in range(self.frame_size - 1):
            delta = target.velocity * self.chirp_time
            if target.dir < 0:
                dists.append(dist - (i+1)*delta)
            elif target.dir > 0:
                dists.append(dist + (i+1)*delta)
        return dists

    def plot_range_resolution(self):
        """
        Purpose:
            Plot range resolution vs bandwidth.
        Input:
            N/A
        Output:
            N/A
        """

        # Sweep for smooth curve
        bandwidths = np.linspace(0.5e9, 15e9, 200)
        range_resolutions = self.c / (2 * bandwidths)

        # Current bandwidth
        current_bw = self.bandwidth

        fig, ax = plt.subplots(figsize=(10, 6))

        # Range-resolution curve
        ax.plot(
            bandwidths / 1e9,
            range_resolutions,
            linewidth=2,
            color="blue",
            label="Range Resolution Curve"
        )

        # Vertical lines for all bandwidth options
        for bw in self.bandwidth_arr:
            if bw == current_bw:
                # Red line for current bandwidth
                ax.axvline(
                    bw / 1e9,
                    color='red',
                    linestyle='--',
                    linewidth=2,
                    label=f"Current: {bw/1e9:.1f} GHz"
                )
            else:
                # Orange line for other options
                ax.axvline(
                    bw / 1e9,
                    color='orange',
                    linestyle='--',
                    linewidth=1.5,
                    alpha=0.7
                )

        # Axis labels / formatting
        ax.set_xlabel("Bandwidth (GHz)", fontsize=12)
        ax.set_ylabel("Range Resolution (m)", fontsize=12)
        ax.set_title("Range Resolution vs Bandwidth\nΔR = c / 2B", fontsize=14, fontweight="bold")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)

        # Improved vertical scaling
        all_res = [self.c / (2 * bw) for bw in self.bandwidth_arr] + list(range_resolutions)
        ymin = min(all_res) * 0.8
        ymax = max(all_res) * 1.1
        ax.set_ylim([ymin, ymax])

        plt.tight_layout()
        plt.savefig("RangeResPlot.png", dpi=150)
        plt.close(fig)

        print("** Generated Range Resolution Plot @ RangeResPlot.png")

    def plot_velocity_resolution(self):
        """
        Purpose:
            Plot velocity resolution for each (starting_freq, bandwidth) pair.
            Each pair i corresponds to each pair of configurations
        Input:
            N/A
        Output:
            N/A
        """

        chirp_counts = np.linspace(4, 256, 200)  # sweep number of chirps
        fig, ax = plt.subplots(figsize=(10, 6))

        for i, (f0, bw) in enumerate(zip(self.freq_arr, self.bandwidth_arr)):
            lam = self.c / f0
            T_frame = chirp_counts * self.chirp_time
            vel_res = lam / (2 * T_frame)

            # Plot the velocity resolution line
            ax.plot(
                chirp_counts,
                vel_res,
                linewidth=1,
                label=f"f0={f0/1e9:.1f} GHz, B={bw/1e9:.1f} GHz"
            )

        # Add vertical dashed line at current frame size
        ax.axvline(
            self.frame_size,
            color='red',
            linestyle='--',
            linewidth=2,
            label=f'Current Frame Size = {self.frame_size}'
        )

        # Formatting
        ax.set_xlabel("Number of Chirps per Frame", fontsize=12)
        ax.set_ylabel("Velocity Resolution (m/s)", fontsize=12)
        ax.set_title("Velocity Resolution vs Frame Size\nΔv = λ / (2 * chirp_time)",
                     fontsize=14, fontweight="bold")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
        plt.tight_layout()

        plt.savefig("VelocityResPlot.png", dpi=150)
        plt.close(fig)
        print("** Generated Velocity Resolution Plot @ VelocityResPlot.png")

    def plot_chirp(self, tx, rx):
        """
        Purpose:
            Plots Tx and Rx sawtooth chirps to show the frequency varying over
            time as well as a small time delay between the signals
            NOTE: The difference is small so you might need to zoom in on the  plot
        Input:
            tx, rx
        Output:
            N/A
        """
        t = np.linspace(0, 2 * self.chirp_time, len(tx))
        fig, ax = plt.subplots(2, 1, figsize=(7,6), constrained_layout=True)
        tx = np.array(tx)
        rx = np.array(rx)
        diff = np.diff(rx)
        rx[0:np.where(diff <= 0)[0][0]+1] = 0
        tx[-1] = 1
        # Main plot: Tx and Rx
        ax[0].plot(t*1e6, tx, label="Tx")
        ax[0].plot(t*1e6, rx, label="Rx", alpha=0.7)
        ax[0].set_xlabel("Time (µs)")
        ax[0].set_ylabel("Amplitude")
        ax[0].set_title("FMCW Sawtooth Chirps")
        ax[0].legend()
        ax[0].grid(True)
        # Beat frequency plot
        ax[1].plot(t*1e6, tx - rx, color="r")
        ax[1].set_xlabel("Time (µs)")
        ax[1].set_ylabel("Amplitude")
        ax[1].set_title("Beat Frequency Signal")
        ax[1].set_ylim([-0.005, 0.01])
        ax[1].grid(True)

        plt.savefig("ChirpPlot.png", dpi=150)
        plt.close(fig)
        
        print("** Generated Chirp Plot @ ChirpPlot.png")