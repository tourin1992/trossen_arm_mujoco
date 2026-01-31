# Coding Conventions

**Analysis Date:** 2026-01-31

## Naming Patterns

**Files:**
- Lowercase with underscores: `sim_env.py`, `scripted_policy.py`, `record_sim_episodes.py`
- Constants file: `constants.py`
- Utilities file: `utils.py`
- Package init: `__init__.py`

**Functions:**
- Lowercase with underscores (snake_case): `sample_box_pose()`, `get_observation_base()`, `make_sim_env()`, `plot_observation_images()`
- Private/internal functions indicated by leading underscore (not commonly used in codebase)
- Test/demo functions prefixed with `test_`: `test_sim_teleop()`, `test_ee_sim_env()`, `test_policy()`

**Variables:**
- Lowercase with underscores: `env_state`, `left_trajectory`, `onscreen_render`, `task_config`
- Private instance variables prefixed with underscore (rarely used in this codebase)
- Constants (in `constants.py`) use UPPERCASE: `START_ARM_POSE`, `DT`, `ASSETS_DIR`, `BOX_POSE`
- Temporary loop variables: `t`, `i`, `cam`, `ts` (timestep), `ts_first`

**Types/Classes:**
- PascalCase: `TrossenAIStationaryTask`, `TransferCubeTask`, `BasePolicy`, `PickAndTransferPolicy`, `TrossenAIStationaryEETask`
- Task classes inherit from `base.Task` and use "Task" suffix
- Policy classes inherit from `BasePolicy` and use "Policy" suffix

**Module-level Constants:**
- All caps with underscores: `ROOT_DIR`, `SIM_TASK_CONFIGS`, `DT`, `START_ARM_POSE`, `ASSETS_DIR`, `BOX_POSE`
- Organized in `trossen_arm_mujoco/constants.py`

## Code Style

**Formatting:**
- Tool: Ruff (with ruff-format)
- Configured in `.pre-commit-config.yaml`
- Line length target: 99 characters (isort config, ruff target)
- Actual line lengths observed: 82-117 characters (some files exceed target)

**Linting:**
- Tool: Ruff
- Configuration in `.pre-commit-config.yaml` with `--fix` flag
- Additional tools:
  - `pyupgrade` (v3.19.1) - modernizes Python syntax
  - `mypy` (v0.931) - type checking (though type hints not heavily used)
  - `codespell` (v2.4.1) - spell checking
  - Pre-commit hooks: trailing whitespace, merge conflicts, YAML validation

**Ruff Focus Areas:**
- Default ruff rules applied
- `ruff-format` applied automatically

## Import Organization

**Order:**
1. Standard library imports: `import os`, `import collections`, `import argparse`, `import time`
2. Third-party imports: `import numpy as np`, `from dm_control import mujoco`, `import matplotlib.pyplot as plt`, `import h5py`
3. Local application imports: `from trossen_arm_mujoco.constants import ...`, `from trossen_arm_mujoco.utils import ...`

**Path Aliases:**
- No path aliases observed
- Direct relative imports from `trossen_arm_mujoco` package used consistently

**Example from `sim_env.py`:**
```python
import collections

from dm_control.mujoco.engine import Physics
from dm_control.suite import base
import matplotlib.pyplot as plt
import numpy as np

from trossen_arm_mujoco.constants import BOX_POSE, START_ARM_POSE
from trossen_arm_mujoco.utils import (
    get_observation_base,
    make_sim_env,
    plot_observation_images,
)
```

**Tool Configuration:**
- isort profile: "black"
- isort line_length: 99
- isort force_sort_within_sections: true
- isort lexicographical: true

## Error Handling

**NotImplementedError Pattern:**
- Used to mark abstract methods that must be implemented in subclasses
- Examples in `sim_env.py:155` and `ee_sim_env.py:121`
- Pattern: `raise NotImplementedError` with docstring describing requirement

```python
def get_reward(self, physics: Physics) -> int:
    """..."""
    raise NotImplementedError
```

**Validation:**
- Minimal validation observed
- Example: `assert BOX_POSE[0] is not None` (sim_env.py:190)
- File operations check existence: `if not os.path.exists(...)` then `os.makedirs(...)`

**Exception Handling:**
- No explicit try-except blocks observed in main source code
- Let exceptions propagate naturally (file not found, missing imports, etc.)

## Logging

**Framework:** `print()` statements for basic output

**Patterns:**
- Used in test/demo functions: `test_sim_teleop()`, `test_policy()`, `test_ee_sim_env()`
- Example from `scripted_policy.py:148`:
```python
print(f"Generate trajectory for {box_xyz=}")
```
- F-string formatting with walrus operator when helpful
- Progress tracking: `from tqdm import tqdm` used in `record_sim_episodes.py` for progress bars

**When to Log:**
- Episode status and progress: `print(f"Episode {episode_idx + 1}/{num_episodes}")`
- Success/failure outcomes: `print(f"{episode_idx=} Successful, {episode_return=}")`
- Variable inspection during development: `print(f"Generate trajectory for {box_xyz=}")`

## Comments

**When to Comment:**
- Explain non-obvious logic, especially in complex simulation or trajectory code
- Mark action parsing sections: `# left_arm_action = action[:6]` followed by brief description
- Document workflow steps: `# lifted`, `# attempted transfer`, `# successful transfer`
- Mark state transitions or important constants

**Example from `sim_env.py:71-83`:**
```python
left_arm_action = action[:6]
right_arm_action = action[8 : 8 + 6]
normalized_left_gripper_action = action[6]
normalized_right_gripper_action = action[8 + 6]

# Assign the processed gripper actions
left_gripper_action = normalized_left_gripper_action
right_gripper_action = normalized_right_gripper_action

# Ensure both gripper fingers act oppositely
full_left_gripper_action = [left_gripper_action, left_gripper_action]
```

**Docstrings:**
- Tool: Python docstrings (formatted for sphinx-compatible rendering)
- Format: Standard Python docstring style with `:param`, `:return:`, `:raises` tags
- Location: All public functions and classes include docstrings
- Example from `utils.py:43-58`:

```python
def sample_box_pose() -> np.ndarray:
    """
    Generate a random pose for a cube within predefined position ranges.

    :param: [if applicable]
    :return: A 7D array containing the sampled position ``[x, y, z, w, x, y, z]`` representing the
        cube's position and orientation as a quaternion.
    """
```

**Type Hints:**
- Observed in function signatures: `def __init__(self, random: int | None = None, onscreen_render: bool = False, cam_list: list[str] = [])`
- Modern Union syntax: `int | None` (Python 3.10+)
- Container types: `list[str]`, `dict`, `np.ndarray`
- Return types specified: `-> np.ndarray`, `-> dict`, `-> int`
- Docstrings use `:param type:` notation even when type hints present

## Function Design

**Size:**
- Small to medium functions preferred
- `sample_box_pose()`: 16 lines - utility function
- `get_observation_base()`: 15 lines - wrapper around physics.render()
- `get_reward()` methods: 30-40 lines - state machine-like logic
- Longer functions for complex workflows: `record_sim_episodes.py:main()` spans multiple logical sections

**Parameters:**
- Keyword arguments with defaults for configuration: `onscreen_render: bool = False`, `cam_list: list[str] = []`
- Positional arguments for core data: `physics: Physics`, `action: np.ndarray`
- Default empty list `[]` used (though mutable default anti-pattern present)
- Related parameters grouped together: physics operations, rendering options, camera lists

**Return Values:**
- Single return: `np.ndarray`, `dict`, `int`, `list[AxesImage]`
- Functions return modified/derived data consistently
- Collections.OrderedDict used for observation returns to maintain key order

**Example Function from `utils.py:82-117`:**
```python
def make_sim_env(
    task_class: base.Task,
    xml_file: str = "trossen_ai_scene.xml",
    task_name: str = "sim_transfer_cube",
    onscreen_render: bool = False,
    cam_list: list[str] = [],
):
    """Docstring with param/return specs."""
    if "sim_transfer_cube" in task_name:
        assets_path = os.path.join(ASSETS_DIR, xml_file)
        physics = mujoco.Physics.from_xml_path(assets_path)
        task = task_class(
            random=False,
            onscreen_render=onscreen_render,
            cam_list=cam_list,
        )
    else:
        raise NotImplementedError(f"Task {task_name} is not implemented.")

    return control.Environment(...)
```

## Module Design

**Exports:**
- No explicit `__all__` definitions in any module
- Public API clear from imports in other modules
- `__init__.py` (28 lines) contains only copyright header - no re-exports

**Barrel Files:**
- `__init__.py` does not aggregate exports
- Direct imports from submodules required: `from trossen_arm_mujoco.utils import ...`

**Module Organization:**
- Functional modules: `utils.py` contains utility functions
- Constant modules: `constants.py` contains all configuration
- Task/Policy modules: `sim_env.py`, `ee_sim_env.py`, `scripted_policy.py` contain related classes
- Scripts: `trossen_arm_mujoco/scripts/` contains runnable demonstrations and utilities

---

*Convention analysis: 2026-01-31*
