# RadarSim: FMCW Radar Simulation

**Authors:** Josh Kushner & Michael Zhou  
**Course:** ECE 239AS Implementation Final Project  

A 3D interactive radar simulation built with **Panda3D** that demonstrates **Frequency-Modulated Continuous Wave (FMCW)** radar signal processing and target detection. Users can spawn moving targets, adjust radar parameters, and visualize how these changes affect range and velocity measurements in real time.

---

## Demo


https://github.com/user-attachments/assets/7e6cc36c-2805-4c78-806e-3eb3209fafef



---

## Features

### Interactive 3D Environment
- Real-time 3D visualization with orbital camera controls  
- Clickable target spawning with configurable velocities  
- Visual line-of-sight indicators  
- Automatic back-and-forth target motion  

### Configurable Radar Parameters
- **Starting Frequency:** 100–120 GHz  
- **Bandwidth:** 2.0–3.0 GHz  
- **Chirp Time:** 40–50 μs  
- **Frame Size:** 40–50 chirps per frame  
- **Target Velocity:** Dynamically calculated  

### Signal Processing
- FMCW chirp generation with realistic delays  
- Low-pass filtering for beat signal extraction  
- FFT-based range estimation  
- Doppler processing for velocity estimation  
- Range-Doppler Map (RDM) generation  

### Visualization & Analysis
- **Range FFT:** Detected target ranges with peak annotations  
- **Range-Doppler Map:** 2D heatmap of range vs. velocity  
- **Range Resolution Plot:** Shows effect of bandwidth on range resolution (ΔR = c/2B)  
- **Velocity Resolution Plot:** Shows effect of frame size on velocity resolution (Δv = λ/2T)  
- **Chirp Plot:** Transmit/receive waveforms and beat signal  

---

## Installation

### Requirements
- Python >= 3.8  
- numpy, scipy, matplotlib, panda3d  

### Setup
```bash
# Clone the repository
git clone git@github.com:josh-kushner/RadarSim.git
cd RadarSim

# Install dependencies
pip install -r requirements.txt

# Run the simulation
python main.py
```

## Usage

### Controls

**Setup Phase (State 0)**
- `1-5` – Cycle through radar parameters (frequency, bandwidth, chirp time, frame size, target velocity)
- `Right Click` – Spawn target
- `Backspace` – Clear all targets
- `Left Mouse Hold` – Rotate camera
- `Space` – Start simulation

**Run Phase (State 1)**
- `Left Mouse Hold` – Rotate camera
- `Space` – Stop simulation and generate plots

**End Phase (State 2)**
- View generated plots (`PNG` files)
- `Space` – Reset simulation

---

### Workflow
1. Configure radar parameters using number keys (1-5)
2. Spawn targets by right-clicking on the ground plane
3. Press `Space` to start the simulation
4. Press `Space` again to stop and automatically generate signal processing plots

---

### Output Files
- `RangeFFT.png` – Range FFT showing detected target peaks
- `RangeDopplerMap.png` – 2D range-velocity heatmap
- `RangeResPlot.png` – Range resolution vs. bandwidth analysis
- `VelocityResPlot.png` – Velocity resolution vs. frame size analysis
- `ChirpPlot.png` – Transmit/receive waveforms and beat signal

---

### Architecture
- `scene.py` – Main simulation environment and user interface
- `signals.py` – FMCW signal processing and DSP algorithms
- `actor.py` – Target object definitions
- `camera.py` – Camera control system

---

### Limitations
- Frame rate limitations require distance interpolation (cannot update at 25 kHz)
- Targets spawn only in positive X-Y quadrant
- Targets move only along line-of-sight paths
- Maximum visualization range: 150 m
- Single-transmitter, single-receiver configuration

---

### Future Enhancements
- Physics-based wave propagation
- Object mesh collisions for finer detail
- Interactive target movement
- Live plotting of signals and graphs
- 3D angle estimation (azimuth/elevation)

---

### Educational Value
RadarSim demonstrates:
- Range and velocity estimation
- Beat frequency generation
- Range-Doppler mapping
- Effects of radar parameters on resolution

