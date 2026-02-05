# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 Humble project for camera integration. The repository is currently in early development.

## Build Commands

Once ROS 2 packages are added, use standard ROS 2 Humble build commands:

```bash
# Build entire workspace (run from workspace root)
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with symlink install (faster iteration for Python)
colcon build --symlink-install
```

## Testing

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# View test results
colcon test-result --verbose
```

## Environment Setup

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace overlay (after building)
source install/setup.bash
```

## Architecture

This repository will contain ROS 2 packages for camera drivers and integration. Structure will follow standard ROS 2 conventions with package.xml and CMakeLists.txt (C++) or setup.py (Python) per package.
