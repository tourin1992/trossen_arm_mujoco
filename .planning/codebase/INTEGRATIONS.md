# External Integrations

**Analysis Date:** 2026-01-31

## APIs & External Services

**Hardware Control:**
- Trossen AI Robot (wxai_v0 model)
  - What it's used for: Real robot arm control for motion replay
  - SDK/Client: `trossen_arm` Python package
  - Auth: IP address-based (network communication to robot control interface)
  - File: `trossen_arm_mujoco/scripts/replay_episode_real.py`
  - Configuration: IP addresses passed as command-line arguments (`--left_ip`, `--right_ip`)

**Physics Simulation:**
- MuJoCo Physics Engine
  - What it's used for: Bimanual robotic arm manipulation simulation with contact dynamics
  - SDK/Client: `dm_control` (DeepMind Control Suite wrapper)
  - File: `trossen_arm_mujoco/sim_env.py`, `trossen_arm_mujoco/ee_sim_env.py`
  - Integration: Direct XML-based scene loading from `trossen_arm_mujoco/assets/`

## Data Storage

**Databases:**
- Not used - No relational or document database detected

**File Storage:**
- Local filesystem only
  - Episode data: HDF5 format stored in `~/.trossen/mujoco/data/` (configured in `constants.py`)
  - Asset files: XML scene files and STL mesh models in `trossen_arm_mujoco/assets/`
  - Client: h5py for HDF5 read/write operations
  - Files affected: `trossen_arm_mujoco/scripts/record_sim_episodes.py`, `trossen_arm_mujoco/scripts/visualize_eps.py`

**HDF5 Episode Structure:**
- Format: Hierarchical Data Format (version 5)
- Contents: Per episode stored as `episode_{idx}.hdf5`
  - `action` - Action trajectory array [timesteps, action_dim]
  - `observation` - Multi-camera images and proprioceptive state
  - Stored at path: `{ROOT_DIR}/{data_dir}/episode_{idx}.hdf5`
- Used by: `record_sim_episodes.py` (write), `replay_episode_real.py` (read), `visualize_eps.py` (read)

**Caching:**
- Not implemented - No caching layer detected

## Authentication & Identity

**Auth Provider:**
- Not applicable - No authentication system
- Hardware access: IP address-based network access control only

## Monitoring & Observability

**Error Tracking:**
- Not detected - No error tracking service integrated

**Logs:**
- Console logging via `print()` statements throughout codebase
  - Files: `trossen_arm_mujoco/scripts/record_sim_episodes.py` (lines 29-46), `replay_episode_real.py` (lines 37-42)
  - Includes: Episode progress, configuration parameters, position differences, trajectory generation info
  - No centralized logging framework detected

**Debug Visualization:**
- Real-time matplotlib visualization in simulation
  - Multiple camera feeds rendered during episode execution
  - Used in: `trossen_arm_mujoco/utils.py` (`plot_observation_images()`, `set_observation_images()`)

## CI/CD & Deployment

**Hosting:**
- Not applicable - Pure library/toolkit for local development

**CI Pipeline:**
- Pre-commit hooks configured (`.pre-commit-config.yaml`)
  - Tools: codespell, isort, pyupgrade, ruff (linting and formatting)
  - No automated testing or deployment pipeline detected

**Pre-commit Tools:**
- Pre-commit framework v5.0.0 for git hooks
- Codespell v2.4.1 - Spell checking
- isort v6.0.1 - Import sorting
- pyupgrade v3.19.1 - Python syntax modernization
- ruff v0.11.3 - Linting and formatting
- mypy v0.931 - Type checking (configured but older version)

## Environment Configuration

**Required env vars:**
- Not required - No environment variables detected in code
- Configuration via: Command-line arguments and hardcoded constants in `constants.py`

**Secrets location:**
- Not applicable - No secrets management detected
- Robot IP addresses passed at runtime (not stored)

**Command-line Configuration:**
- Data directory: `--data_dir` (record_sim_episodes.py)
- Root directory override: `--root_dir` (record_sim_episodes.py, replay_episode_real.py)
- Episode selection: `--episode_idx` (replay_episode_real.py)
- Camera names: `--cam_names` (record_sim_episodes.py)
- Robot IPs: `--left_ip`, `--right_ip` (replay_episode_real.py)
- Rendering: `--onscreen_render` flag

## Webhooks & Callbacks

**Incoming:**
- Not applicable - No webhooks detected

**Outgoing:**
- Not applicable - No outgoing webhooks detected

## Real Hardware Integration

**Device Communication:**
- Trossen AI Robot Control:
  - Protocol: Network-based (IP address)
  - Configuration approach: `trossen_arm.TrossenArmDriver()` in `replay_episode_real.py`
  - Connection details:
    ```python
    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_leader,
        ip_address,
        True,
    )
    ```
  - Hardware setup: Dual arm configuration (left and right)
  - Control modes: Position mode (`trossen_arm.Mode.position`)
  - Files: `trossen_arm_mujoco/scripts/replay_episode_real.py`

**Simulation to Hardware:**
- Episode replay pipeline:
  1. Collect joint trajectories in simulation via `TransferCubeEETask`
  2. Record actions to HDF5 in `record_sim_episodes.py`
  3. Replay recorded actions on real hardware via `replay_episode_real.py`
  - Gripper actuation: Separate from arm joints (indices [6,7] for left, [14,15] for right)

## Asset Management

**Scene Files:**
- `trossen_ai_scene.xml` - Mocap-based control (used in `ee_sim_env.py`)
- `trossen_ai_scene_joint.xml` - Joint-controlled simulation (used in `sim_env.py`)
- `trossen_ai.xml` - Base model definition
- Location: `trossen_arm_mujoco/assets/`
- Loaded via: `dm_control.mujoco.Physics.from_xml_path()`

**Mesh Models:**
- STL and OBJ files in `trossen_arm_mujoco/assets/meshes/`
- Referenced by XML scene files
- Contains: Robot components, cameras, environmental objects (tables, boxes)

---

*Integration audit: 2026-01-31*
