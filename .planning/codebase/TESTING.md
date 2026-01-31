# Testing Patterns

**Analysis Date:** 2026-01-31

## Test Framework

**Status:**
- No dedicated test framework configured (pytest, unittest, etc.)
- No test infrastructure detected in `pyproject.toml`
- No pytest.ini, setup.cfg, or tox.ini files

**Current Testing Approach:**
- Manual testing via function-level demonstrations
- Test functions defined as `test_*()` functions called directly via `if __name__ == "__main__"`
- No automated test runner

**Development Tools Configured:**
- mypy (v0.931) - type checking enabled in pre-commit hooks
- ruff - linting and formatting
- codespell - spell checking
- pyupgrade - Python syntax modernization

## Test File Organization

**Location:**
- Test/demo functions embedded in source modules
- Not separated into dedicated test directory
- Live alongside production code in `trossen_arm_mujoco/` package
- Scripts directory contains demo utilities: `trossen_arm_mujoco/scripts/`

**Naming Convention:**
- Test functions use `test_` prefix: `test_sim_teleop()`, `test_ee_sim_env()`, `test_policy()`
- Typically located at end of module after class definitions
- Executed conditionally under `if __name__ == "__main__":`

**Examples:**
- `trossen_arm_mujoco/sim_env.py:247-275` - `test_sim_teleop()` function
- `trossen_arm_mujoco/ee_sim_env.py:258-281` - `test_ee_sim_env()` function
- `trossen_arm_mujoco/scripted_policy.py:310-417` - `test_policy()` function with extensive parameterization

## Test Structure

**Demo Function Pattern:**

```python
def test_sim_teleop():
    """
    Runs a simulation to test teleoperation with the Trossen AI robotic arms.
    Testing teleoperation in sim with Trossen AI.
    Requires hardware and Trossen AI repo to work.
    """
    # setup the environment
    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(TransferCubeTask, "trossen_ai_scene_joint.xml")
    ts = env.reset()
    episode = [ts]
    # setup plotting
    plt_imgs = plot_observation_images(ts.observation, cam_list)

    for t in range(1000):
        action = np.random.uniform(-np.pi, np.pi, 16)
        ts = env.step(action)
        episode.append(ts)

        # update plot
        plt_imgs[0].set_data(ts.observation["images"]["cam_high"])
        # ... more camera updates

        plt.pause(0.02)
```

**Typical Workflow (Executable Test):**
1. Environment setup: `make_sim_env(TaskClass, config)`
2. Episode initialization: `env.reset()`
3. Step loop: `for t in range(episode_len):`
4. Action generation: policy, random, or hardcoded
5. Observation collection: `ts = env.step(action)`
6. Optional visualization: plotting, rendering
7. Post-episode analysis: success rates, returns

**Setup/Teardown:**
- Implicit setup in test function body
- No explicit teardown (relying on Python garbage collection)
- matplotlib figures may be closed: `plt.close()` (example in `scripted_policy.py:350`)

## Mocking

**Current Approach:**
- No explicit mocking framework detected
- Direct use of actual dependencies:
  - MuJoCo physics simulator (`dm_control`)
  - Matplotlib rendering for visualization
  - File I/O for data storage (h5py in `record_sim_episodes.py`)

**Simulation as Mock:**
- MuJoCo XML files serve as configurable test environments
- Physics engine provides realistic simulation without mocking
- Camera rendering actual rather than stubbed

**Example with Controlled Variability:**
From `scripted_policy.py:test_policy()`:
```python
# Use real environment with controlled policy
env = make_sim_env(TransferCubeEETask, task_name=task_name, ...)
for episode_idx in range(num_episodes):
    ts = env.reset()
    policy = PickAndTransferPolicy(inject_noise)  # Noise injection optional
    for step in range(episode_len):
        action = policy(ts)
        ts = env.step(action)
```

## Fixtures and Test Data

**Test Data Organization:**
- MuJoCo XML files in `trossen_arm_mujoco/assets/`
- Example configurations: `trossen_ai_scene.xml`, `trossen_ai_scene_joint.xml`
- Constants defined in `trossen_arm_mujoco/constants.py`

**Fixture Pattern (Constants as Fixtures):**
From `constants.py`:
```python
START_ARM_POSE = [
    0.0,
    np.pi / 12,
    # ... 14 joint positions
]

DT = 0.02  # timestep

SIM_TASK_CONFIGS = {
    "sim_transfer_cube": {
        "num_episodes": 1,
        "episode_len": 600,
        "onscreen_render": False,
        "inject_noise": False,
        "cam_names": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
    }
}
```

**Trajectory Fixtures:**
From `scripted_policy.py` - hardcoded waypoints serve as expected behavior:
```python
self.left_trajectory = [
    {
        "t": 0,
        "xyz": init_mocap_pose_left[:3],
        "quat": init_mocap_pose_left[3:],
        "gripper": 0,
    },  # sleep
    # ... additional waypoints
]
```

**Location:**
- Fixtures scattered through modules as constants and hardcoded values
- No centralized fixture factory or conftest pattern

## Coverage

**Requirements:**
- Not enforced (no coverage tools in dependencies or config)
- mypy type checking enabled but not strict mode
- Pre-commit hooks don't enforce coverage thresholds

**Observed Coverage Gaps:**
- Error handling paths rarely tested (few try-except blocks in source)
- Edge cases (empty camera lists, missing files) not explicitly tested
- Multi-episode robustness not systematically verified
- Policy interaction with invalid actions not validated

## Test Types

**Demonstration Tests (Most Common):**
- `test_sim_teleop()` - Basic environment teleoperation with random actions
- `test_ee_sim_env()` - End-effector space control test
- `test_policy()` - Policy execution with success rate tracking
- Purpose: Verify components work together; generate demonstration data

**Behavioral Tests (Implicit):**
- Success/failure detection in `test_policy()`:
  ```python
  episode_return = np.sum([ts.reward for ts in episode[1:]])
  if episode_return > 0:
      print(f"{episode_idx=} Successful, {episode_return=}")
  ```

**Integration Tests (Embedded in Scripts):**
- `record_sim_episodes.py` - Full episode recording pipeline
- `visualize_eps.py` - Data playback and analysis
- `ee_circle_demo.py` - Circular motion demonstration

**Unit Tests:**
- Not explicitly present
- Small utilities like `sample_box_pose()` used implicitly in integration tests

**E2E Tests:**
- Multi-episode recording and playback in `record_sim_episodes.py`
- Requires actual MuJoCo simulation and asset files

## Common Patterns

**Async Testing:**
- Not applicable (no async code in codebase)
- Synchronous step-by-step simulation used throughout

**Error/Edge Case Testing:**
```python
# Validation pattern from make_sim_env()
if "sim_transfer_cube" in task_name:
    # ... setup
else:
    raise NotImplementedError(f"Task {task_name} is not implemented.")
```

**Success Criteria Definition:**
From `TransferCubeTask.get_reward()` (sim_env.py:206-244):
```python
reward = 0
if touch_right_gripper:
    reward = 1
# lifted
if touch_right_gripper and not touch_table:
    reward = 2
# attempted transfer
if touch_left_gripper:
    reward = 3
# successful transfer
if touch_left_gripper and not touch_table:
    reward = 4
return reward
```

**Observation Validation:**
From `get_observation_base()` (utils.py:61-79):
```python
obs: collections.OrderedDict = collections.OrderedDict()
if on_screen_render:
    obs["images"] = dict()
    for cam in cam_list:
        obs["images"][cam] = physics.render(height=480, width=640, camera_id=cam)
return obs
```

**Random Action Generation:**
Used in `test_sim_teleop()` (sim_env.py:262):
```python
action = np.random.uniform(-np.pi, np.pi, 16)
```

**Policy-Based Testing:**
From `test_policy()` (scripted_policy.py:343-349):
```python
policy = PickAndTransferPolicy(inject_noise)
for step in range(episode_len):
    action = policy(ts)
    ts = env.step(action)
    episode.append(ts)
    if onscreen_render:
        plt_imgs = set_observation_images(ts.observation, plt_imgs, cam_list)
```

## Running Tests

**Current Execution:**
- Directly run Python files with test functions
  ```bash
  python trossen_arm_mujoco/sim_env.py
  python trossen_arm_mujoco/ee_sim_env.py
  python trossen_arm_mujoco/scripted_policy.py
  ```

**With Parameters:**
- Scripted policy tests support command-line arguments:
  ```bash
  python -m trossen_arm_mujoco.scripted_policy \
    --num_episodes 5 \
    --episode_len 600 \
    --onscreen_render \
    --inject_noise
  ```

**Data Generation:**
- `record_sim_episodes.py` used to generate demonstration datasets:
  ```bash
  python -m trossen_arm_mujoco.scripts.record_sim_episodes \
    --task_name sim_transfer_cube \
    --num_episodes 10
  ```

## Testing Recommendations

**Current Gaps:**
- No automated test runner (pytest/unittest)
- No CI/CD integration visible
- No test organization (mixing demo code with validation)
- Limited assertion-based validation
- No mocking for isolated unit testing

**Where to Add Tests:**
- New test suite: `tests/` directory with pytest
- Utility function validation: `tests/test_utils.py`
- Task class behavior: `tests/test_tasks.py`
- Policy execution: `tests/test_policies.py`

---

*Testing analysis: 2026-01-31*
