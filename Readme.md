# Traffic Management System

## Overview

This project simulates a **Traffic Management System** designed to monitor and optimize traffic flow in a city. It integrates key components like vehicle routing, congestion monitoring, traffic signal management, and disruption handling. The system supports real-time updates and dynamic adjustments to manage traffic efficiently.

---

## Features

### City Traffic Network

- Representation of intersections and roads.
- Dynamic addition/removal of intersections and roads.
- Real-time road status updates (e.g., open, blocked).
- Loading network data from CSV files.

### Vehicle Routing System

- Manages vehicles and their routes.
- Prioritizes emergency vehicles using A\* algorithm.
- Recalculates routes dynamically during disruptions.
- Supports CSV-based vehicle data loading.

### Congestion Monitor

- Tracks traffic density on roads.
- Highlights congested roads.
- Displays congestion levels with visual indicators.

### Traffic Signal Management

- Configurable traffic signals with multiple phases.
- Emergency mode activation for prioritized vehicle movement.
- Dynamic signal timing adjustments based on traffic density.

### Disruption Manager

- Handles road blockages and repairs.
- Triggers route recalculations for affected vehicles.
- Updates road statuses dynamically.

### Traffic Simulation

- Interactive menu-driven interface.
- Real-time updates on vehicles, signals, and disruptions.
- Adjustable simulation speed and support for state saving/loading.

---

## Requirements

- **C++ Compiler**: Compatible with C++11 or later.
- **CSV Files**:
  - `road_network.csv`: Contains road and intersection data.
  - `traffic_signals.csv`: Traffic signal configurations.
  - `vehicles.csv`: List of vehicles and their routes.
  - `road_closures.csv`: Describes disruptions (e.g., blockages, repairs).

---

## Setup

1. **Compile the Code**:
   ```bash
   g++ -o TrafficSimulation Source.cpp
   ```
