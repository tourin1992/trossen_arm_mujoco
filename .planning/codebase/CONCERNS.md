# Codebase Concerns

**Analysis Date:** 2026-01-31

## Tech Debt

**Hardcoded Magic Numbers Throughout Codebase:**
- Issue: Camera resolution, gripper positions, action indices, and pose values are scattered as magic numbers throughout the code
- Files: `trossen_arm_mujoco/utils.py`, `trossen_arm_mujoco/sim_env.py`, `trossen_arm_mujoco/ee_sim_env.py`, `trossen_arm_mujoco/scripted_policy.py`, `trossen_arm_mujoco/constants.py`
- Examples:
  - Camera resolution hardcoded as `480, 640` in `utils.py:78`
  - Gripper position `0.044` repeated in `constants.py:57` and throughout `scripted_policy.py`
  - Z-range clamped to single value `[0.0125, 0.0125]` in `utils.py:52`
  - Action slicing indices like `action[:6]`, `action[8:8+6]`, `action[8+6]` in `sim_env.py:71-74`
  - Mocap position constants in `ee_sim_env.py:98-102`
  - Quaternion constants in `scripted_policy.py` (e.g., `np.array([1, 0, 0, 0])`)
- Impact: Difficult to maintain, prone to out-of-sync errors if dimensions change, limits configurability
- Fix approach: Extract all magic numbers to `constants.py` with descriptive names (e.g., `CAMERA_HEIGHT`, `CAMERA_WIDTH`, `GRIPPER_OPEN_POS`, `GRIPPER_CLOSED_POS`, `ACTION_LEFT_ARM_INDICES`, `MOCAP_LEFT_INIT_POS`)

**Incomplete Environment Configuration Pattern:**
- Issue: `BOX_POSE` is initialized as `[None]` in `constants.py:72` and modified externally as a side-effect in `record_sim_episodes.py:139`
- Files: `trossen_arm_mujoco/constants.py:72`, `trossen_arm_mujoco/scripts/record_sim_episodes.py:139-141`, `trossen_arm_mujoco/sim_env.py:190-191`
- Impact: Mutable global state, hard to track state changes, makes testing and reusability difficult, non-deterministic behavior
- Fix approach: Pass `box_pose` as an explicit parameter through environment initialization chain rather than modifying global state

**Unused or Incomplete Docstrings:**
- Issue: `ee_sim_env.py:110` has duplicate docstring - one in method signature and one empty repeated below it
- Files: `trossen_arm_mujoco/ee_sim_env.py:110`
- Impact: Code clarity, maintenance burden
- Fix approach: Remove duplicate docstring

**Import Location Inconsistency:**
- Issue: `BOX_POSE` is imported from `sim_env.py` in `record_sim_episodes.py:41` but should be imported from `constants.py` where it's actually defined
- Files: `trossen_arm_mujoco/scripts/record_sim_episodes.py:41`
- Current: `from trossen_arm_mujoco.sim_env import BOX_POSE, TransferCubeTask`
- Should be: `from trossen_arm_mujoco.constants import BOX_POSE`
- Impact: Confuses code navigation, creates hidden dependencies
- Fix approach: Import `BOX_POSE` directly from `constants.py`

---

## Known Bugs

**Z-Range Clamping Bug in sample_box_pose():**
- Symptoms: Objects sampled with `sample_box_pose()` always have identical z-coordinates (0.0125)
- Files: `trossen_arm_mujoco/utils.py:50-58`
- Code:
  ```python
  z_range = [0.0125, 0.0125]  # Both min and max are the same!
  ranges = np.vstack([x_range, y_range, z_range])
  cube_position = np.random.uniform(ranges[:, 0], ranges[:, 1])
  ```
- Trigger: Every call to `sample_box_pose()` used in `ee_sim_env.py:203`
- Impact: Objects never vary in height, reduces task variability, may cause training data distribution issues
- Workaround: None - the bug is systematic
- Fix approach: Define proper z-range (e.g., `[0.0, 0.05]` or similar) as a constant

**Action Index Boundaries Risk in sim_env.py:**
- Symptoms: Potential IndexError if action array size is incorrect or mismatched
- Files: `trossen_arm_mujoco/sim_env.py:71-74`
- Code:
  ```python
  left_arm_action = action[:6]        # Indices 0-5
  right_arm_action = action[8 : 8 + 6]  # Indices 8-13
  normalized_left_gripper_action = action[6]    # Index 6
  normalized_right_gripper_action = action[8 + 6]  # Index 14
  ```
- Expects exactly 16 elements (indices 0-15) but no validation
- Trigger: Passing action array with wrong dimensions
- Impact: Silent failures or cryptic IndexError messages
- Fix approach: Add assertion: `assert len(action) == 16, f"Expected action size 16, got {len(action)}"` at start of `before_step()`

---

## Performance Bottlenecks

**Inefficient Contact Pair Search in Reward Computation:**
- Problem: Linear O(n) search through all contact pairs for each reward computation
- Files: `trossen_arm_mujoco/sim_env.py:213-220`, `trossen_arm_mujoco/ee_sim_env.py:228-235`
- Code:
  ```python
  all_contact_pairs = []
  for i_contact in range(physics.data.ncon):
      id_geom_1 = physics.data.contact[i_contact].geom1
      id_geom_2 = physics.data.contact[i_contact].geom2
      name_geom_1 = physics.model.id2name(id_geom_1, "geom")
      name_geom_2 = physics.model.id2name(id_geom_2, "geom")
      contact_pair = (name_geom_1, name_geom_2)
      all_contact_pairs.append(contact_pair)
  ```
- Called every step during episode execution
- Impact: Accumulates list garbage, repeats string lookups, minor but measurable overhead at 50+ steps/episode
- Improvement path: Cache geometry name mappings, use sets for O(1) lookups, or use geometry indices directly with constants

**Matplotlib Rendering in Training Loop:**
- Problem: `plt.pause(0.02)` in `utils.py:183` blocks execution during visualization
- Files: `trossen_arm_mujoco/utils.py:183`, `trossen_arm_mujoco/scripts/record_sim_episodes.py:113`
- Impact: Visualization during data collection slows down episode recording by ~50ms per frame
- Improvement path: Use non-blocking visualization or decouple from training loop

---

## Fragile Areas

**Task Base Classes with Unimplemented Methods:**
- Files: `trossen_arm_mujoco/sim_env.py:147-155`, `trossen_arm_mujoco/ee_sim_env.py:114-121` and `163-170`
- Why fragile: Base classes (`TrossenAIStationaryTask`, `TrossenAIStationaryEETask`) define `get_reward()` and `get_env_state()` that raise `NotImplementedError`. Subclasses must override. If a new subclass forgets, silent failure occurs until `get_reward()` is called.
- Safe modification: Add runtime validation in environment initialization or add abstract method decorators using `abc` module
- Test coverage: No tests to verify all task classes properly implement required methods

**Hard-Coded Action Space Assumptions:**
- Files: `trossen_arm_mujoco/sim_env.py:71-74`, `trossen_arm_mujoco/ee_sim_env.py:71-86`
- Why fragile: Code assumes exact action dimensions (16 elements for joint control, 23 elements for EE control) with no validation. Changing robot DOF requires updates in multiple places.
- Safe modification: Define `ACTION_DIM` constants per task and validate in `before_step()`
- Test coverage: No tests of action processing

**Episode State Management in Policy:**
- Files: `trossen_arm_mujoco/scripted_policy.py:93-131`
- Why fragile: `BasePolicy` maintains mutable trajectory lists that are popped during execution. If episode is interrupted or replayed, state becomes corrupted. `step_count` is never reset.
- Safe modification: Copy trajectories instead of mutating them; add explicit `reset()` method; validate waypoint ordering
- Test coverage: No tests of policy state transitions

**Global State in record_sim_episodes.py:**
- Files: `trossen_arm_mujoco/scripts/record_sim_episodes.py:139-141`
- Why fragile: Modifies global `BOX_POSE` list between environment instances. If parallel processing added, causes race conditions.
- Safe modification: Use environment-local configuration or pass through constructor
- Test coverage: No tests

---

## Security Considerations

**Unsafe File Path Handling:**
- Risk: Hardcoded home directory expansion `~/.trossen/mujoco/data/` without validation
- Files: `trossen_arm_mujoco/constants.py:34`, `trossen_arm_mujoco/scripts/record_sim_episodes.py:59-60`
- Current mitigation: Implicit - only runs locally
- Recommendations: Validate paths are within expected boundaries; use `Path.expanduser()` with checks; add warnings if writing outside project directory

**Missing Input Validation on Quaternions:**
- Risk: Quaternions from scripted policy (`pyquaternion.Quaternion`) are converted to numpy arrays without validation
- Files: `trossen_arm_mujoco/scripted_policy.py:150-153`, `169`
- Example: `gripper_pick_quat = gripper_pick_quat * Quaternion(axis=[0.0, 1.0, 0.0], degrees=-45)` - no bounds checking
- Current mitigation: None
- Recommendations: Validate quaternion norm is ~1.0 before sending to physics engine

---

## Test Coverage Gaps

**No Unit Tests:**
- What's not tested: All core modules lack unit tests
- Files: `trossen_arm_mujoco/sim_env.py`, `trossen_arm_mujoco/ee_sim_env.py`, `trossen_arm_mujoco/utils.py`, `trossen_arm_mujoco/scripted_policy.py`
- Risk: Refactoring breaks hidden assumptions; bugs in core logic go undetected
- Priority: High

**No Action Processing Tests:**
- What's not tested: `TrossenAIStationaryTask.before_step()` and `TrossenAIStationaryEETask.before_step()`
- Files: `trossen_arm_mujoco/sim_env.py:64-93`, `trossen_arm_mujoco/ee_sim_env.py:64-86`
- Risk: Action dimension changes silently break execution
- Priority: High

**No Reward Function Tests:**
- What's not tested: Contact-based reward logic
- Files: `trossen_arm_mujoco/sim_env.py:206-244`, `trossen_arm_mujoco/ee_sim_env.py:220-255`
- Risk: Reward computation bugs affect all training/evaluation
- Priority: High

**No Utility Function Tests:**
- What's not tested: `sample_box_pose()` bounds, `get_observation_base()`, image rendering
- Files: `trossen_arm_mujoco/utils.py`
- Risk: The z-range bug (above) would be caught immediately
- Priority: High

**No Policy State Tests:**
- What's not tested: Trajectory interpolation, policy reset, multi-episode execution
- Files: `trossen_arm_mujoco/scripted_policy.py`
- Risk: Policy corruption in complex scenarios
- Priority: Medium

**No Integration Tests:**
- What's not tested: Full episode execution from reset to termination
- Risk: Issues only surface during data collection (expensive)
- Priority: Medium

---

## Missing Critical Features

**No Episode Length Termination:**
- Problem: Episodes don't terminate when reaching `time_limit`
- Files: `trossen_arm_mujoco/utils.py:110-117` (environment created with `time_limit=20`)
- Blocks: Cannot train RL agents that need proper episode boundaries
- Impact: Data collection runs indefinitely without time limit enforcement

**No Error Handling in Environment Reset:**
- Problem: No try-catch or validation if XML file is missing or physics initialization fails
- Files: `trossen_arm_mujoco/utils.py:99-108`
- Blocks: Poor error messages when XML assets are missing
- Impact: Cryptic failures for new users

**No Configuration Validation:**
- Problem: Task configs in `SIM_TASK_CONFIGS` are not validated against actual environment parameters
- Files: `trossen_arm_mujoco/constants.py:38-46`
- Blocks: Typos in config keys silently use defaults
- Impact: Hard to debug configuration issues

---

## Scaling Limits

**Single Task Implementation:**
- Current capacity: Only `sim_transfer_cube` task implemented
- Limit: Adding new tasks requires modifying `sim_env.py`, `ee_sim_env.py`, `scripted_policy.py`, and `utils.py`
- Scaling path: Refactor to task registry pattern; move task-specific logic to separate task modules

**Fixed Camera List:**
- Current capacity: Hardcoded to 4 cameras
- Limit: Layout logic in `plot_observation_images()` handles 4-camera special case; adding camera breaks subplot layout
- Scaling path: Make subplot layout fully dynamic (already partially done but has hardcoded 4-camera case)

---

## Dependencies at Risk

**dm_control Dependency:**
- Risk: Heavy external dependency on DeepMind Control Suite; small breaking changes could affect entire codebase
- Impact: All simulation logic depends on dm_control API
- Migration plan: Encapsulate dm_control imports in `utils.py` layer to ease future migration to other simulators

**trossen_arm Package:**
- Risk: Undocumented external dependency listed in `pyproject.toml:21`
- Impact: Installation fails if package unavailable; no version pinning
- Migration plan: Add explicit version constraints; document availability

---

## Code Quality Issues

**TODO Comment Indicating Incomplete Design:**
- Location: `trossen_arm_mujoco/sim_env.py:186-187`
- Content: "TODO Notice: this function does not randomize the env configuration. Instead, set BOX_POSE from outside reset qpos, control and box position"
- Impact: Environment randomization is delegated to external configuration rather than proper design pattern

**Inconsistent Docstring Format:**
- Issue: Some methods use RST format (`:param:`, `:return:`, `:raises:`), others are incomplete
- Files: Throughout codebase
- Impact: Harder to generate automated documentation

**Dead Code in Docstrings:**
- Location: `trossen_arm_mujoco/ee_sim_env.py:110` - duplicate docstring
- Impact: Maintenance confusion

---

*Concerns audit: 2026-01-31*
