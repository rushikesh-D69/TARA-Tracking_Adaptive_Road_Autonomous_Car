# CARLA ADAS Simulation System

A comprehensive Advanced Driver Assistance Systems (ADAS) implementation validated in the CARLA autonomous driving simulator. This project demonstrates a **"Sim-to-Real" workflow**, validating real-time safety features and algorithms optimized for unstructured environments before hardware deployment.

![Main View](src/IMG-20251127-WA0010.jpg)

## 🚀 Overview

TARA bridges the gap between software simulation and hardware deployment. It implements a modular Python-based ADAS stack that processes sensor data (Camera, LiDAR, Radar) to execute safety-critical maneuvers in a realistic simulation environment. The goal is to validate the control logic in CARLA before porting the stack to a Raspberry Pi/Jetson Nano platform for the 1:10 scale physical prototype.
##  Features

### Core ADAS Systems

- **Forward Collision Warning (FCW)**: Real-time collision risk assessment with Time-to-Collision (TTC) calculation
- **Automatic Emergency Braking (AEB)**: Autonomous emergency braking system to prevent collisions
- **Adaptive Cruise Control (ACC)**: Intelligent speed control that maintains safe distance from lead vehicles
- **Lane Departure Warning (LDW)**: Monitors lane position and alerts when vehicle drifts from lane center
- **Blind Spot Detection (BSD)**: Detects vehicles in blind spots on both sides
- **Traffic Sign Recognition (TSR)**: Recognizes and displays current speed limits
- **Intelligent Overtaking**: Automated overtaking system with safety checks
- **Advanced Lane Detection**: Enhanced lane detection and tracking
- **Sensor Visualization**: Real-time visualization of sensor data and ADAS alerts
![Alt text](src/IMG-20251127-WA0003.jpg)

### Additional Features

- **Scenario Manager**: Predefined driving scenarios for testing
- **Multi-sensor Integration**: Camera, LiDAR, GPS, and collision sensors
- **Real-time HUD**: Comprehensive heads-up display with ADAS status
- **Traffic Generation**: Configurable traffic vehicle spawning
- **Weather Simulation**: Dynamic weather conditions
- **Synchronous Mode**: Deterministic simulation for testing

## 📋 Requirements

- Python 3.6+
- CARLA Simulator (0.9.x or later)
- Pygame
- NumPy
- Matplotlib
- Open3D (optional, for LiDAR visualization)
- Pillow

##  Installation

1. **Install CARLA Simulator**
   ```bash
   # Download CARLA from https://github.com/carla-simulator/carla/releases
   # Extract to your desired location
   ```

2. **Install Python Dependencies**
   ```bash
   pip install -r requirements.txt
   # or
   pip3 install -r requirements.txt
   ```

3. **Set up CARLA Python API**
   ```bash
   # Ensure CARLA Python API is in your Python path
   # The script automatically searches for carla module
   ```

##  Usage

### Quick Start

**Step 1: Start CARLA Simulator**

Open a terminal and navigate to your CARLA installation directory, then start the CARLA server:

```bash
cd carla_simulator
./CarlaUE4.sh -quality-level=Low -prefernvidia -nosound
```

**Step 2: Run the ADAS Simulation**

In a new terminal, navigate to the examples directory and run the simulation:

```bash
cd carla_simulator/PythonAPI/examples
python3 tara.py --sync --agent Behavior --loop -n 30
```

This command will:
- Run in synchronous mode (`--sync`)
- Use the Behavior agent (`--agent Behavior`)
- Loop to new destinations automatically (`--loop`)
- Spawn 30 traffic vehicles (`-n 30`)

### Basic Usage

```bash
python3 tara.py
```

### Command Line Options

![Alt text](src/IMG-20251127-WA0012.jpg)

```bash
python tara.py [OPTIONS]

Options:
  -v, --verbose          Print debug information
  --host H               IP of the host server (default: 127.0.0.1)
  -p, --port P           TCP port to listen to (default: 2000)
  --tm-port P            Port to communicate with Traffic Manager (default: 8000)
  --res WIDTHxHEIGHT     Window resolution (default: 1280x720)
  --sync                 Enable synchronous mode execution
  --filter PATTERN       Actor filter (default: "vehicle.*")
  --generation G         Actor generation: "1", "2", or "All" (default: "2")
  -l, --loop             Sets new random destination upon reaching previous one
  -a, --agent AGENT      Agent type: "Behavior", "Basic", or "Constant" (default: "Behavior")
  -b, --behavior BEHAVIOR  Agent behavior: "cautious", "normal", or "aggressive" (default: "normal")
  -s, --seed SEED        Set seed for repeating executions
  -n, --num-vehicles N   Number of traffic vehicles to spawn (default: 50)
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `F1` | Toggle ADAS system ON/OFF |
| `F2` | Toggle Adaptive Cruise Control (ACC) |
| `F3` | Toggle Automatic Emergency Braking (AEB) |
| `ESC` / `Ctrl+Q` | Quit simulation |
| `TAB` | Cycle through camera views |
| `0-9` | Switch camera positions |
| `` ` `` | Toggle sensor visualization |

### Example: Running with Custom Settings

```bash
# Recommended: Run with synchronous mode, Behavior agent, looping, and 30 vehicles
python3 tara.py --sync --agent Behavior --loop -n 30

# Run with 100 traffic vehicles, aggressive behavior, synchronous mode
python3 tara.py --sync -n 100 -b aggressive --res 1920x1080

# Run with debug output and custom seed
python3 tara.py -v -s 42 --agent Behavior

# Run with more traffic vehicles for intensive testing
python3 tara.py --sync --agent Behavior --loop -n 50
```

## 📸 Screenshots

### Main Simulation View
![Main Simulation](src/IMG-20251127-WA0010.jpg)
*Main simulation view showing the vehicle with ADAS systems active*

### ADAS HUD Display
![ADAS HUD](src/IMG-20251127-WA0006.jpg)
*Real-time ADAS status display showing FCW, ACC, LDW, and other system states*

### Forward Collision Warning
![FCW Alert](src/IMG-20251127-WA0009.jpg)
*Forward Collision Warning system detecting an obstacle ahead with TTC calculation*

### Adaptive Cruise Control
![ACC Active](screenshots/acc_active.png)
*Adaptive Cruise Control maintaining safe distance from lead vehicle*


### Sensor Visualization
![Sensor View](src/IMG-20251127-WA0004.jpg)
*Multi-sensor visualization showing camera, LiDAR, and GPS data*


---
![LIDAR VIEW](src/IMG-20251127-WA0011.jpg)



## 🏗️ Architecture

### System Components

```
tara.py
├── ADASManager          # Central ADAS system coordinator
├── AdaptiveCruiseControl # ACC implementation
├── ForwardCollisionWarning # FCW implementation
├── AutomaticEmergencyBraking # AEB implementation
├── LaneDepartureWarning # LDW implementation
├── BlindSpotDetection   # BSD implementation
├── TrafficSignRecognition # TSR implementation
├── IntelligentOvertaking # Overtaking system
├── ScenarioManager      # Scenario testing framework
├── SensorVisualizationManager # Sensor data visualization
├── World                # CARLA world management
├── HUD                  # Heads-up display
└── CameraManager        # Camera sensor management
```

### ADAS System Flow

1. **Sensor Data Collection**: Camera, LiDAR, GPS, and collision sensors gather real-time data
2. **ADAS Processing**: Each ADAS module processes sensor data independently
3. **Risk Assessment**: Systems calculate collision risks, lane positions, and traffic conditions
4. **Control Integration**: ADASManager integrates all systems and modifies vehicle control
5. **Visualization**: HUD displays real-time ADAS status and alerts

##  Technical Details

### Forward Collision Warning (FCW)
- Calculates Time-to-Collision (TTC) with vehicles ahead
- Three alert levels: None, Warning, Critical
- Detection range: 50 meters
- Considers relative velocity and closing speed

### Adaptive Cruise Control (ACC)
- Maintains safe following distance based on time gap
- Adjusts speed relative to lead vehicle
- Configurable target speed and time gap
- Detection range: 50 meters

### Automatic Emergency Braking (AEB)
- Activates when collision is imminent (TTC < 1.0s)
- Applies maximum braking force
- Overrides driver input for safety
- Always active by default (can be toggled for testing)

### Lane Departure Warning (LDW)
- Monitors lateral offset from lane center
- Uses CARLA map data for lane detection
- Warns when vehicle drifts beyond threshold
- Calculates distance to lane boundaries

##  Performance

- Real-time processing at 20-30 FPS (depending on hardware)
- Low latency ADAS response (< 100ms)
- Efficient sensor data processing
- Optimized collision detection algorithms

##  Testing

The system includes a scenario manager for testing various driving conditions:

- Emergency braking scenarios
- Lane change scenarios
- Overtaking scenarios
- Traffic intersection scenarios
- 

##  Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

##  License

This project is based on CARLA's automatic control example, which is licensed under the MIT License. See the original copyright notice in the source code.


## ⚠️ Important Notes

- **CARLA Server Must Be Running**: Always start the CARLA simulator first before running the Python script
- **Terminal 1**: Run `./CarlaUE4.sh -quality-level=Low -prefernvidia -nosound` to start CARLA
- **Terminal 2**: Run `python3 tara.py --sync --agent Behavior --loop -n 30` to start the ADAS simulation
- **Synchronous Mode**: The `--sync` flag ensures deterministic simulation and is recommended for testing
- **Performance**: Use `-quality-level=Low` for better performance on lower-end systems

