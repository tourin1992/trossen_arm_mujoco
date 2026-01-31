# Technology Stack

**Analysis Date:** 2026-01-31

## Languages

**Primary:**
- Python 3.10+ - All simulation, policy, and scripting logic

**Secondary:**
- XML (MuJoCo configuration) - Robot physics and scene definitions
- URDF (robot description format) - Robot kinematics and geometry

## Runtime

**Environment:**
- Python 3.10 or higher (specified in `pyproject.toml` as `requires-python = ">=3.10"`)

**Package Manager:**
- pip - Python package installation
- Lockfile: Not detected (uses `pyproject.toml` for dependency specification)

## Frameworks

**Core:**
- dm_control - MuJoCo physics simulation wrapper and control framework
  - `dm_control.mujoco` - Physics engine integration
  - `dm_control.suite.base` - Task base class for RL environments
  - `dm_control.rl.control` - Environment wrapper for timestep management
  - `dm_env` - DeepMind environment interface for timestep definition

**Visualization:**
- matplotlib - Plotting and real-time camera image visualization (`matplotlib.pyplot`)

**Robotics:**
- pyquaternion - Quaternion math for 3D orientation manipulation
- trossen_arm - Hardware driver integration for real robot control (optional for hardware playback)

**Data & Utilities:**
- numpy - Numerical operations and array manipulation
- h5py - HDF5 file format for episode recording and replay
- tqdm - Progress bars for data collection loops
- opencv-python - Computer vision utilities

**Build/Dev:**
- setuptools>=61.0 - Package building and distribution

## Key Dependencies

**Critical:**
- dm_control - Enables MuJoCo physics simulation and reinforcement learning environment interface
- dm_env - Defines the `TimeStep` interface used throughout observation and action handling
- numpy - Core numerical library for trajectory generation, pose interpolation, and robot control calculations

**Infrastructure:**
- h5py - Stores simulation episodes with action, observation, and state data for policy training
- matplotlib - Renders multi-camera observations for debugging and visualization
- pyquaternion - Performs rotation interpolation between waypoints in scripted policies
- trossen_arm - Hardware driver for real robot integration (required only for `replay_episode_real.py`)
- tqdm - Progress indication for data collection (used in `record_sim_episodes.py`)
- opencv-python - Image processing support (imported but extent of use unclear)

## Configuration

**Environment:**
- No `.env` file detected
- Configuration parameters stored in `trossen_arm_mujoco/constants.py`:
  - `ROOT_DIR` = `~/.trossen/mujoco/data/` - Default directory for HDF5 episode data
  - `ASSETS_DIR` - Resolved dynamically via `importlib.resources` pointing to `trossen_arm_mujoco/assets/`
  - `SIM_TASK_CONFIGS` - Dictionary defining task parameters (episodes, duration, cameras, rendering)
  - `START_ARM_POSE` - Default robot joint configuration (8 joints per arm)
  - `DT` = 0.02 - Control timestep (50 Hz frequency)

**Build:**
- `pyproject.toml` - Single configuration source for dependencies, metadata, and tool settings
- Build system: setuptools with PEP 517/518 compliance

## Platform Requirements

**Development:**
- Python 3.10+ installation
- Linux/Unix system (MuJoCo cross-platform but typically run on Linux for robotics)
- Display server for rendering (X11 or compatible)
- Virtual environment recommended (conda as per README)

**Production/Hardware:**
- Trossen AI robot hardware (wxai_v0 model)
- Network connectivity to robot control IP (for `replay_episode_real.py`)
- Real-time control capable system for actual hardware playback
- MuJoCo rendering disabled for headless simulation on servers

**Simulation:**
- MuJoCo physics engine (included as dependency via dm_control)
- Asset files in `trossen_arm_mujoco/assets/` directory
- XML scene files: `trossen_ai_scene.xml` (mocap-based) or `trossen_ai_scene_joint.xml` (joint-controlled)

## Dependencies Version Summary

From `pyproject.toml`:
- dm_control - (version unspecified, latest compatible)
- dm_env - (version unspecified)
- h5py - (version unspecified)
- matplotlib - (version unspecified)
- numpy - (version unspecified)
- tqdm - (version unspecified)
- opencv-python - (version unspecified)
- pyquaternion - (version unspecified)
- trossen_arm - (version unspecified, custom Trossen package)

Note: `pyproject.toml` uses flexible version specifiers (no version constraints listed), allowing pip to install latest compatible versions.

---

*Stack analysis: 2026-01-31*
