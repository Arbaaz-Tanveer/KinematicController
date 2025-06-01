# KinematicController
This repository contains a controller based on kinematic equations for robust path following. It also provides setup instructions, dependencies installation, and a usage guide for running the simulation package with ROS 2.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)

   * [System Dependencies](#system-dependencies)
   * [Python Packages](#python-packages)
   * [Open Motion Planning Library (OMPL)](#open-motion-planning-library-ompl)
3. [Building the Package](#building-the-package)
4. [Usage Guide](#usage-guide)

---

## Prerequisites

* Ubuntu 22 or later
* ROS 2 (Foxy, Galactic, or Humble) installed and sourced
* `cmake`, `git`, and `build-essential` packages

---

## Installation

### System Dependencies

Update package lists and upgrade existing packages:

```bash
sudo apt update && sudo apt upgrade -y
````

Install development libraries:

```bash
sudo apt install -y \
    libopencv-dev \
    libboost-all-dev \
    libompl-dev ompl-demos \
    libeigen3-dev
```

### Python Packages

Install required Python modules:

```bash
pip install pygame
```

### Open Motion Planning Library (OMPL)

If not already installed via `libompl-dev`, you can build OMPL from source. However, `libompl-dev` is usually sufficient. For a source build:

Download and run the OMPL installation script:

```bash
wget [https://ompl.kavrakilab.org/install-ompl-ubuntu.sh](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh)
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh
```

*(Note: The `libompl-dev` package installed via apt should typically suffice. The script method is an alternative if a specific version or custom build is needed.)*

-----

## Building the Package

In your ROS 2 workspace (e.g., `~/KinematicController`), build the package:

```bash
cd ~/KinematicController
colcon build --symlink-install # Optional: --symlink-install can be convenient
source install/setup.bash
```

-----

## Usage Guide

To run the system, you will need to launch the simulation and the controller nodes. Open separate terminals for each command.

1.  **Start the Bot Simulation:**

    ```bash
    ros2 run simulation_pkg bot_simulation
    ```

2.  **Run the Kinematic Controller:**

    ```bash
    ros2 run simulation_pkg kinematic_controller
    ```

3.  **Run the Main Node (from package 'o1'):**

    ```bash
    ros2 run o1 main1
    ```

### Sending a Target Position

To send a target position and orientation to the bot, you can publish to the `o1/decision_target_data` topic. For example, to send the bot to X=7.0, Y=-3.0, and angle 1 radian:

```bash
ros2 topic pub o1/decision_target_data std_msgs/msg/Float32MultiArray "{data: [7.0, -3.0, 1.0]}"
```


-----


