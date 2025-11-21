# Vertical Navigation (VNAV) Simulator

This project is a C++17 console program that simulates a simple VNAV (Vertical Navigation) system for an aircraft.  
It models VNAV modes, basic climb/descent logic, and logs the flight profile to a CSV file for analysis.

## Features

- VNAV modes:
  - `OFF`
  - `ARMED`
  - `CLIMB`
  - `PATH` (descent)
  - `ALT_CAP` (altitude capture)
  - `ALT_HOLD` (altitude hold)
- Random waypoint constraints (altitude + speed)
- Simple flight envelope limits (Mach and low-altitude speed)
- Great-circle distance calculation between airports
- CSV logging to `vnav_log.csv`:
  - step
  - VNAV mode
  - current altitude
  - selected altitude
  - commanded vertical speed
  - commanded speed
  - remaining distance (NM)

## Build

From WSL in the project directory:

```bash
g++ -std=c++17 Vertical_Navigation_System.cpp -o vnav
