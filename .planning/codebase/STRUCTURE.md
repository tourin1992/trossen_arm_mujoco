# Codebase Structure

**Analysis Date:** 2026-01-31

## Directory Layout

```
trossen_arm_mujoco/
├── trossen_arm_mujoco/          # Main package
│   ├── __init__.py              # Package initialization
│   ├── constants.py             # Global configuration and constants
│   ├── sim_env.py               # Joint-space task definitions
│   ├── ee_sim_env.py            # End-effector space task definitions
│   ├── scripted_policy.py       # Policy implementations
│   ├── utils.py                 # Shared utilities
│   ├── assets/                  # Physics and rendering assets
│   │   ├── meshes/              # 3D geometry (STL, OBJ, textures)
│   │   ├── trossen_ai_bimanual.xml
│   │   ├── trossen_ai_scene.xml
│   │   ├── trossen_ai_scene_joint.xml
│   │   ├── trossen_ai_scene_2.xml
│   │   ├── trossen_ai_joint.xml
│   │   └── wxai_follower.xml
│   └── scripts/                 # Executable scripts
│       ├── record_sim_episodes.py    # Data collection
│       ├── visualize_eps.py          # Episode playback
│       ├── ee_circle_demo.py         # Circular motion demo
│       ├── snapshot.py               # Static scene rendering
│       └── replay_episode_real.py    # Hardware integration
├── pyproject.toml               # Package metadata and dependencies
├── .pre-commit-config.yaml      # Code quality hooks
└── README.md                    # Documentation
```

## Directory Purposes

**trossen_arm_mujoco/:**
- Purpose: Main Python package containing all simulation and control logic
- Contains: Core modules for task definition, policy execution, utilities, and assets
- Key files: sim_env.py, ee_sim_env.py, scripted_policy.py

**trossen_arm_mujoco/assets/:**
- Purpose: Physics simulation assets and rendering resources
- Contains: MuJoCo XML scene definitions and 3D model files
- Generated: No (all manually created)
- Committed: Yes

**trossen_arm_mujoco/assets/meshes/:**
- Purpose: 3D geometry for rendering and collision
- Contains: STL meshes (robot links, grippers, frame, table), OBJ models (tabletop, tablelegs), textures (PNG)
- Generated: No
- Committed: Yes

**trossen_arm_mujoco/scripts/:**
- Purpose: Entry point scripts for specific workflows
- Contains: Data collection, visualization, hardware testing scripts
- Key files: record_sim_episodes.py (primary), visualize_eps.py, snapshot.py

## Key File Locations

**Entry Points:**
- `trossen_arm_mujoco/sim_env.py`: test_sim_teleop() for joint-space testing
- `trossen_arm_mujoco/ee_sim_env.py`: test_ee_sim_env() for EE-space testing
- `trossen_arm_mujoco/scripted_policy.py`: test_policy() function with CLI args
- `trossen_arm_mujoco/scripts/record_sim_episodes.py`: main() for data collection (primary workflow)

**Configuration:**
- `pyproject.toml`: Package name, version, dependencies, tool configs
- `trossen_arm_mujoco/constants.py`: DT, START_ARM_POSE, SIM_TASK_CONFIGS, ASSETS_DIR, BOX_POSE

**Core Logic:**
- `trossen_arm_mujoco/sim_env.py`: Joint-space task definitions (TrossenAIStationaryTask, TransferCubeTask)
- `trossen_arm_mujoco/ee_sim_env.py`: EE-space task definitions (TrossenAIStationaryEETask, TransferCubeEETask)
- `trossen_arm_mujoco/scripted_policy.py`: Policy base class and PickAndTransferPolicy implementation
- `trossen_arm_mujoco/utils.py`: Environment creation (make_sim_env), observation utilities, visualization

**Testing:**
- No dedicated test directory. Tests are test_*() functions in module files:
  - `sim_env.py::test_sim_teleop()`
  - `ee_sim_env.py::test_ee_sim_env()`
  - `scripted_policy.py::test_policy()`

**Assets:**
- XML scenes: `trossen_arm_mujoco/assets/trossen_ai_scene.xml` (primary with EE constraint), `trossen_ai_scene_joint.xml` (for joint replay)
- Base model: `trossen_arm_mujoco/assets/trossen_ai_bimanual.xml` (included by scene XMLs)

## Naming Conventions

**Files:**
- Scripts: `snake_case.py` (record_sim_episodes.py, ee_circle_demo.py)
- Modules: `snake_case.py` (sim_env.py, scripted_policy.py)
- Assets: `lowercase_with_underscores.xml` (trossen_ai_scene.xml)

**Directories:**
- Python: `lowercase_no_underscore` (trossen_arm_mujoco)
- Subdirectories: `lowercase_no_underscore` (assets, scripts, meshes)

**Classes:**
- Base/abstract: PascalCase (Task, BasePolicy)
- Concrete task classes: `[Domain][Action]Task` pattern (TransferCubeTask, TransferCubeEETask)
- Concrete policy classes: `[Task]Policy` pattern (PickAndTransferPolicy)

**Functions:**
- snake_case (make_sim_env, get_observation_base, sample_box_pose)
- Test functions: `test_*()` (test_sim_teleop, test_policy)

**Variables:**
- Constants (module-level): UPPER_SNAKE_CASE (DT, START_ARM_POSE, ASSETS_DIR)
- Instance variables: snake_case (self.cam_list, self.step_count)
- Numpy arrays: descriptive lowercase (left_xyz, quat, gripper)

**Types:**
- Observation keys: snake_case (qpos, qvel, env_state, mocap_pose_left)
- Action components: [side]_[component] (left_arm_action, right_gripper_action)

## Where to Add New Code

**New Task:**
- Implementation: `trossen_arm_mujoco/sim_env.py` or `trossen_arm_mujoco/ee_sim_env.py` depending on control space
- Pattern: Inherit from TrossenAIStationaryTask or TrossenAIStationaryEETask, override get_reward() and initialize_episode()
- Example: TransferCubeTask at line 158 in sim_env.py
- Register: Add task name to SIM_TASK_CONFIGS in constants.py

**New Policy:**
- Implementation: `trossen_arm_mujoco/scripted_policy.py`
- Pattern: Inherit from BasePolicy, implement generate_trajectory(ts_first) method
- Example: PickAndTransferPolicy at line 134 in scripted_policy.py
- Import: Add to record_sim_episodes.py line 40

**New Scene/Environment Setup:**
- XML model: `trossen_arm_mujoco/assets/[name].xml`
- Pattern: Include trossen_ai_bimanual.xml or trossen_ai_joint.xml, define cameras and objects
- Reference in scripts: Update xml_file parameter in make_sim_env() calls

**Utilities/Helpers:**
- Location: `trossen_arm_mujoco/utils.py` for general, or create new module in trossen_arm_mujoco/ for specialized
- Example: get_observation_base, plot_observation_images, make_sim_env

**Scripts/Workflows:**
- Location: `trossen_arm_mujoco/scripts/` for new entry points
- Pattern: Create standalone script with main() function and argparse CLI
- Example: record_sim_episodes.py

**Configuration:**
- Shared constants: `trossen_arm_mujoco/constants.py`
- Task-specific configs: Add to SIM_TASK_CONFIGS dict
- Example: DT (timestep), START_ARM_POSE (initial config)

## Special Directories

**assets/:**
- Purpose: Physics and rendering data
- Generated: No (all hand-authored)
- Committed: Yes (required for simulation)
- Usage: Loaded at runtime by physics layer via ASSETS_DIR constant

**scripts/:**
- Purpose: Standalone CLI entry points
- Generated: No
- Committed: Yes
- Usage: Run as: python -m trossen_arm_mujoco.scripts.[script_name]

**meshes/**
- Purpose: 3D collision geometry and visual meshes
- Generated: No (CAD outputs)
- Committed: Yes (required for rendering)
- Subdirectory of: assets/

**__pycache__/:**
- Purpose: Python bytecode cache
- Generated: Yes (automatic by Python)
- Committed: No (in .gitignore)

## Import Structure

**External Dependencies:**
```python
from dm_control import mujoco              # Physics simulation
from dm_control.rl import control          # Environment wrapper
from dm_control.suite import base          # Task base class
from matplotlib import pyplot               # Visualization
import numpy as np                          # Numerical computing
import h5py                                 # Data serialization
from pyquaternion import Quaternion        # Quaternion math
from tqdm import tqdm                       # Progress bars
```

**Internal Imports:**
```python
from trossen_arm_mujoco.constants import (DT, START_ARM_POSE, SIM_TASK_CONFIGS, ASSETS_DIR)
from trossen_arm_mujoco.utils import (make_sim_env, get_observation_base)
from trossen_arm_mujoco.sim_env import TransferCubeTask
from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
from trossen_arm_mujoco.scripted_policy import PickAndTransferPolicy
```

**Pattern:** Absolute imports from trossen_arm_mujoco root package, not relative imports

---

*Structure analysis: 2026-01-31*
