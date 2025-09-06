# 🚀 MultiSense-UAV-Detection

[![Stars](https://img.shields.io/github/stars/HLiu-uOttawa/MultiSense-UAV-Detection?style=social)](https://github.com/HLiu-uOttawa/MultiSense-UAV-Detection/stargazers)
[![Issues](https://img.shields.io/github/issues/HLiu-uOttawa/MultiSense-UAV-Detection)](https://github.com/HLiu-uOttawa/MultiSense-UAV-Detection/issues)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen)](https://www.python.org/)
[![License](https://img.shields.io/github/license/HLiu-uOttawa/MultiSense-UAV-Detection)](./LICENSE)

A **ROS 2–based multi-sensor payload system** for UAV detection, tracking, and collision avoidance. Optimized for Jetson Orin NX and Ubuntu 22.04.

---

## Table of Contents
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Repository Structure](#repository-structure)
- [Configuration](#configuration)
- [Demo](#demo)
- [Roadmap](#roadmap)
- [Known Issues](#known-issues)
- [Contributing](#contributing)
- [License](#license)

---

## Features
- ✅ Real-time **radar + camera sensor fusion**
- ✅ **YOLOv8 / YOLOv10** UAV detection
- ✅ **GM-PHD** multi-object tracking
- ✅ Live visualization with **RViz2 / Foxglove**
- ✅ Designed for **Jetson Orin NX** (CUDA) and desktop Linux

---

## Requirements
- **OS:** Ubuntu 22.04 (Jammy)
- **ROS 2:** Humble Hawksbill
- **Python:** 3.10+
- **CUDA:** NVIDIA CUDA (on Jetson/desktop GPU)  
- Optional tools: **Foxglove Studio**, **RViz2**

> If you use a fresh machine, make sure ROS 2 is installed and sourced before building.

---

## Installation
Clone and build the workspace:

```bash
git clone https://github.com/HLiu-uOttawa/MultiSense-UAV-Detection.git
cd MultiSense-UAV-Detection

# Install dependencies (rosdep will read package.xml files)
sudo apt update
sudo apt install -y python3-colcon-common-extensions
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

## Usage
Run the demo launch:

```bash
source install/setup.bash
ros2 launch bringup demo_launch.py
```

**Notes**
- Use `--params-file` to override default parameters (see `bringup/config/*.yaml`).
- Open **RViz2** or **Foxglove** to visualize streams and tracks.

---

## Repository Structure
```
.
├── sensors/
│   ├── camera/           # Camera driver nodes
│   └── radar/            # Radar driver nodes
├── tracking/             # GM-PHD tracking module
├── yolov8/               # YOLOv8/YOLOv10 detection nodes
├── bringup/              # Launch files & configs
├── docs/                 # Project docs (MkDocs-ready)
└── README.md
```

---

## Configuration
- Default parameter files live under `bringup/config/`.
- Typical overrides:
  - **Model paths** (YOLO weights)
  - **Topic names / QoS profiles**
  - **Sensor calibration** (intrinsics/extrinsics, time offsets)
  - **Tracking** thresholds (gating, birth/death rates)

Example (passing a params file):
```bash
ros2 launch bringup demo_launch.py params_file:=bringup/config/demo_params.yaml
```

---

## Demo
> Replace the image path with your actual screenshot if needed.
![Payload demo](docs/images/payload_demo.png)

---

## Roadmap
- [ ] Multi-camera support
- [ ] ROS 2 bag record/playback integration
- [ ] Enhanced Foxglove dashboards
- [ ] Swarm/multi-UAV scenarios and stress testing

---

## Known Issues
- ⚠️ Radar driver may drop frames under heavy CPU load.
- ⚠️ YOLOv10 half-precision support on Jetson Orin NX can be limited.
- ⚠️ Long-run time synchronization drift between radar and camera.
- ⚠️ Tracking parameters need tuning for fast/agile UAV targets.

See open items in [Issues](https://github.com/HLiu-uOttawa/MultiSense-UAV-Detection/issues).

---

## Contributing
Contributions are welcome!  
- Open an Issue for bugs/feature requests.  
- Submit a Pull Request following conventional commit style if possible.  
- See `CONTRIBUTING.md` (if present) for code style and testing notes.

---

## License
This project is licensed under the [MIT License](./LICENSE).
