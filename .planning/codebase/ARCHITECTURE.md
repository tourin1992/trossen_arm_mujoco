# Architecture

**Analysis Date:** 2026-01-31

## Pattern Overview

**Overall:** Multi-layered environment abstraction pattern with task-based learning framework

**Key Characteristics:**
- Task-based simulation framework using dm_control
- Bimanual robotic manipulation with dual arm control
- End-effector (EE) space and joint space control abstraction layers
- Modular task definition enabling policy training and demonstration collection
- Scripted policy execution for trajectory-based control

## Layers

**Simulation Physics Layer:**
- Purpose: Provides low-level physics simulation via MuJoCo
- Location: `trossen_arm_mujoco/assets/` (XML scene definitions)
- Contains: MuJoCo model files (trossen_ai_scene.xml, trossen_ai_scene_joint.xml, etc.) with robot models, collision geometry, cameras, and dynamics
- Depends on: dm_control physics engine
- Used by: Task layer for physics state access and control

**Task Layer:**
- Purpose: Defines task semantics, reward functions, and observation structure
- Location: `trossen_arm_mujoco/sim_env.py`, `trossen_arm_mujoco/ee_sim_env.py`
- Contains: Base task classes (TrossenAIStationaryTask, TrossenAIStationaryEETask) and concrete task implementations (TransferCubeTask, TransferCubeEETask)
- Depends on: dm_control.suite.base.Task, physics layer
- Used by: Environment layer, policy execution

**Environment Layer:**
- Purpose: Wraps task with control loop, timestep management, and time limits
- Location: `trossen_arm_mujoco/utils.py` (make_sim_env function)
- Contains: dm_control.rl.control.Environment wrapping that implements episode lifecycle (reset, step, time_limit)
- Depends on: Task layer
- Used by: Policy execution scripts, data collection

**Policy Layer:**
- Purpose: Generates actions based on observations
- Location: `trossen_arm_mujoco/scripted_policy.py`
- Contains: BasePolicy (abstract), PickAndTransferPolicy (concrete implementation with trajectory waypoints)
- Depends on: Environment layer observations
- Used by: Training scripts, demonstration collection

**Utilities & Configuration Layer:**
- Purpose: Shared helpers, constants, and asset management
- Location: `trossen_arm_mujoco/constants.py`, `trossen_arm_mujoco/utils.py`
- Contains: Configuration constants (DT, START_ARM_POSE, ROOT_DIR), camera setup, pose sampling, visualization
- Depends on: None (provides dependencies to other layers)
- Used by: All layers

## Data Flow

**Episode Execution Flow:**

1. Environment initialization
   - Load XML physics model from `trossen_arm_mujoco/assets/`
   - Instantiate Task (TransferCubeTask or TransferCubeEETask)
   - Wrap in dm_control.rl.control.Environment

2. Episode reset
   - Call task.initialize_episode() which sets robot pose (START_ARM_POSE) and object pose
   - Returns initial TimeStep with observation

3. Per-timestep loop
   - Policy generates action from current observation
   - Environment calls task.before_step(action) for action processing
   - Physics simulation advances by control_timestep (DT=0.02s)
   - Task computes reward from contact information
   - Return TimeStep with (observation, reward, done)

4. Observation structure
   - Images: Dict of camera renders (cam_high, cam_low, cam_left_wrist, cam_right_wrist)
   - qpos: Joint positions (16 values: 6 left_arm + 2 left_gripper + 6 right_arm + 2 right_gripper)
   - qvel: Joint velocities (16 values)
   - env_state: Object state (box pose for TransferCubeTask)
   - mocap_pose_left/right: End-effector poses (EE space only)

**Data Collection Flow:**

1. EE-space execution (TransferCubeEETask + PickAndTransferPolicy)
   - Policy generates end-effector poses and gripper commands
   - Task.before_step() sets mocap bodies from action
   - Inverse kinematics handled by MuJoCo equality constraints
   - Record joint trajectory from resulting qpos

2. Joint-space replay (TransferCubeTask)
   - Use recorded joint trajectory as direct actions
   - Replay with same object configuration (BOX_POSE)
   - Record all observations including images
   - Save as HDF5 dataset in `~/.trossen/mujoco/data/`

**State Management:**

- **Physics state:** Maintained by MuJoCo (qpos, qvel, contact info)
- **Task state:** Ephemeral per episode (reset on initialize_episode)
- **Policy state:** Waypoint trajectory stored in BasePolicy.{left,right}_trajectory, step_count
- **Global state:** BOX_POSE[0] shared reference used to sync object config between environments

## Key Abstractions

**Task Abstraction:**
- Purpose: Encapsulates reward computation and observation formation
- Examples: `trossen_arm_mujoco/sim_env.py` (TrossenAIStationaryTask, TransferCubeTask)
- Pattern: Inherit from dm_control.suite.base.Task, override get_observation(), get_reward(), initialize_episode(), before_step()

**Control Space Abstraction:**
- Purpose: Decouple control input from execution
- Joint Space: Direct qpos/qvel commands to 16 DOF (two 6-DOF arms + grippers)
- EE Space: Position (xyz) + orientation (quat) + gripper command via mocap bodies with IK constraints

**Observation Abstraction:**
- Purpose: Unified interface to sensor data
- Implemented in: `trossen_arm_mujoco/utils.py` (get_observation_base)
- Returns: OrderedDict with images, qpos, qvel, env_state, mocap poses (EE only)

**Policy Abstraction:**
- Purpose: Encapsulates action generation logic
- Base interface: BasePolicy with __call__(ts) -> action
- Trajectory management: Waypoint interpolation with linear interpolation between (t, xyz, quat, gripper)

## Entry Points

**Simulation Testing:**
- Location: `trossen_arm_mujoco/sim_env.py` (test_sim_teleop function)
- Triggers: python -m trossen_arm_mujoco.sim_env
- Responsibilities: Creates random-action environment, runs 1000 steps, visualizes all 4 cameras

**Policy Testing:**
- Location: `trossen_arm_mujoco/scripted_policy.py` (test_policy function, main script)
- Triggers: python trossen_arm_mujoco/scripted_policy.py [--args]
- Responsibilities: Instantiates PickAndTransferPolicy, runs multiple episodes with rendering, reports success/failure

**Data Collection:**
- Location: `trossen_arm_mujoco/scripts/record_sim_episodes.py` (main function)
- Triggers: python trossen_arm_mujoco/scripts/record_sim_episodes.py --data_dir <dir> [--args]
- Responsibilities:
  - Run EE-space policy (TransferCubeEETask) to collect joint trajectory
  - Replay trajectory in joint-space (TransferCubeTask) while recording full observations
  - Serialize to HDF5 with structure: observations/{images/{cam_name}, qpos, qvel}, action

**Scene Visualization:**
- Location: `trossen_arm_mujoco/scripts/snapshot.py`
- Triggers: python trossen_arm_mujoco/scripts/snapshot.py
- Responsibilities: Renders static scene snapshot from multiple camera angles

**Episode Visualization:**
- Location: `trossen_arm_mujoco/scripts/visualize_eps.py`
- Triggers: python trossen_arm_mujoco/scripts/visualize_eps.py [--args]
- Responsibilities: Loads and replays HDF5 episodes with multi-camera visualization

## Error Handling

**Strategy:** Minimal error handling with assumptions about valid inputs

**Patterns:**
- NotImplementedError for abstract methods (get_reward in base tasks)
- Task string validation with conditional in make_sim_env ("sim_transfer_cube" check)
- Array bounds checks for action processing (a_len = len(action) // 2)
- Assertion on BOX_POSE validity in TransferCubeTask.initialize_episode

## Cross-Cutting Concerns

**Logging:** None (relies on print statements for runtime messages)

**Validation:**
- XML model path construction via importlib.resources
- Asset directory resolution: ASSETS_DIR = files("trossen_arm_mujoco").joinpath("assets")
- Camera list defaults to 4 cameras if empty

**Configuration:**
- Centralized in constants.py: DT, START_ARM_POSE, SIM_TASK_CONFIGS
- Task parameters passed via constructor args (random seed, onscreen_render, cam_list)
- XML scene selection via task_name parameter (xml_file parameter to make_sim_env)

---

*Architecture analysis: 2026-01-31*
