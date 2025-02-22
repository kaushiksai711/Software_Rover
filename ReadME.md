# Software Rover Project

This project contains the software components for a rover, including system initialization, telemetry, movement, terrain mapping, object detection, path planning, and more.

---

## **1️⃣ Initialization**
### 📄 `Software_rover.py`
**Description**: Handles system startup, sensor checks, and communication setup.

- **Inputs**:
  - Power-on signal (manual/remote)
  - Sensor readings (LIDAR, camera, motor, communication)

- **Outputs**:
  - System readiness status (Boolean)
  - Communication link status (Boolean)

---

## **2️⃣ Path Smoothing**
### 📄 `utils.py`
**Description**: Implements path smoothing using gradient descent and enhanced heuristic for path planning.

- **Inputs**:
  - Path (List of tuples representing coordinates)
  - Weight data (float)
  - Weight smooth (float)
  - Tolerance (float)

- **Outputs**:
  - Smoothed path (List of tuples representing coordinates)

---

## **3️⃣ Priority Queue**
### 📄 `priority_queue.py`
**Description**: Implements a priority queue for the D* Lite algorithm, handling lexicographic ordering of keys.

- **Inputs**:
  - Vertices and their priorities

- **Outputs**:
  - Ordered vertices based on priority

---

## **4️⃣ Main Execution**
### 📄 `main.py`
**Description**: Main script to run the D* Lite path planning algorithm with a graphical user interface.

- **Inputs**:
  - Command line arguments (window width, height, cell size, margin)
  - User interactions (mouse clicks, keyboard inputs)

- **Outputs**:
  - Visual representation of the grid and path
  - Updated rover position

---

## **5️⃣ Graphical User Interface**
### 📄 `gui.py`
**Description**: Provides a graphical interface for visualizing the grid, obstacles, and path planning.

- **Inputs**:
  - User interactions (mouse clicks, keyboard inputs)
  - Path data (List of tuples representing coordinates)

- **Outputs**:
  - Visual representation of the grid, obstacles, and path
  - Rover's current position

---

## **6️⃣ Path Planning**
### 📄 `d_star_lite.py`
**Description**: Implements the D* Lite path planning algorithm, integrating obstacle detection and path smoothing.

- **Inputs**:
  - Start and goal positions (tuples)
  - Map data (occupancy grid)

- **Outputs**:
  - Planned path (List of tuples representing coordinates)
  - Smoothed path (List of tuples representing coordinates)
  - Cost maps (g and rhs values)

---

## **7️⃣ Terrain Mapping**
### 📄 `Software_rover.py`
**Description**: Simulates scanning the arena with LIDAR and creating a 3D map.

- **Inputs**:
  - LIDAR data

- **Outputs**:
  - Map data (traversable and non-traversable points)

---

## **8️⃣ Object Detection**
### 📄 `Software_rover.py`
**Description**: Detects obstacles and source/destination points using simulated data.

- **Inputs**:
  - Map data (occupancy grid)

- **Outputs**:
  - Updated map data with detected obstacles
  - Source and destination points (tuples)

---

## **9️⃣ Movement**
### 📄 `Software_rover.py`
**Description**: Handles rover movement commands such as moving forward, backward, and turning.

- **Inputs**:
  - Movement commands

- **Outputs**:
  - Rover movement actions

---

## **🔟 Telemetry**
### 📄 `Software_rover.py`
**Description**: Manages telemetry data transmission to the base station.

- **Inputs**:
  - Mode, sensor data, map data

- **Outputs**:
  - Transmitted telemetry data

---

## **1️⃣1️⃣ Emergency Handling**
### 📄 `Software_rover.py`
**Description**: Performs emergency checks and reinitializes sensors if necessary.

- **Inputs**:
  - Sensor status

- **Outputs**:
  - Emergency status (Boolean)
  - Reinitialized sensors