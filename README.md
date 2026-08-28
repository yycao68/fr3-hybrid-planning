# FR3 Hybrid Planning platform

Real Franka FR3 kinematics + MoveIt 2 Hybrid Planning + MuJoCo execution,
built as the experimental platform for the "Predictive Physical
Realizability" paper (`../predictive_realizability_paper_draft.md`, one
level up — this `ros2_ws/` tree is `.gitignore`'d from that paper repo on
purpose: it's engineering infrastructure, not paper content, so it has its
own git history in *this* directory).

The paper's core claim is a certificate-driven, predictive local planner
(**B3**) that catches a future torque violation before it happens, versus
a reactive baseline (**B2**) that only checks the current instant. This
platform ports both, plus the reduced-order Python reference's Level 0-4
response hierarchy, into real MoveIt 2 Hybrid Planning C++ plugins running
against MuJoCo-simulated FR3 dynamics — not a toy model.

## Status: Phases 0–3c done. Phase 3d (reroute) not started.

## Architecture

```
fr3_mujoco_bringup/     Bring-up: URDF/xacro, MoveIt config, MuJoCo model,
                        launch files, controller config. No governor logic
                        of its own — wires everything else together.
fr3_dynamics/           Shared KDL-based inverse dynamics (mass matrix,
                        Coriolis+gravity, KDL<->MoveIt joint index
                        mapping). Used by B2 and B3 so they share ONE
                        dynamics code path (paper's B2-vs-B3 fairness
                        principle) — this is load-bearing, not incidental.
fr3_b2_local_planner/   B2: reactive baseline. Current-instant-only
                        torque-feasible qddot projection via OSQP.
fr3_b3_local_planner/   B3: predictive certificate + Level 0/1/2/4
                        response (see below). The bulk of the interesting
                        code lives here.
```

`fr3_b3_local_planner` internals:

```
torque_margin_certificate.{hpp,cpp}   certificate.py::m_phys, shared by
                                       both plugin targets.
route_retime_search.{hpp,cpp}         Level 1: retiming transform +
                                       bisection/dense-grid search.
reshape_qp.{hpp,cpp}                  Level 2: OSQP reshape QP, shared by
                                       both plugin targets.
horizon_trajectory_operator.cpp       TrajectoryOperatorInterface plugin:
                                       exposes a real receding horizon
                                       (stock SimpleSampler only exposes
                                       one waypoint) + owns the ROUTE-LEVEL
                                       Level 1/2 decision (once per new
                                       trajectory).
b3_constraint_solver.cpp              LocalConstraintSolverInterface
                                       plugin: certificate check + ONLINE
                                       Level 0/2/4 decision (every cycle).
```

Third-party trees (`franka_description`, `moveit_hybrid_planning`,
`mujoco_ros2_control`, `mujoco_menagerie_sparse`) are `.gitignore`'d here —
each is its own clone with its own upstream history. Two of them carry
local-only patches (not pushed, no push access to upstream):
- `moveit_hybrid_planning`: `@loader_path` install-RPATH fix (see below).
- `mujoco_ros2_control`: real-time pacing patch (sleeps off sim-vs-wall-
  clock drift each outer-loop iteration).

## The Level 0–4 hierarchy (paper Sec. V-B, `code/local_planner.py`)

| Level | What | Where | Cadence |
|---|---|---|---|
| 0 | Pass nominal through | both plugins | — |
| 1 | Retime (slow down by λ∈[1,λ_max]) | `HorizonTrajectoryOperator` | once per new route |
| 2 | Reshape (QP over qddot) | both, route- and cycle-level | route: once/route; online: every cycle |
| 3 | Reroute (candidate selection) | **not implemented** | — |
| 4 | Sticky brake | `B3ConstraintSolver` | every cycle, once triggered, forever |

Ordering per route: **retime → reshape → (reroute, unimplemented)**, tried
only if the previous one didn't fix things. Online, per cycle: **reshape →
brake**. This exactly mirrors `code/local_planner.py::plan_route` /
`online_step`'s own cascade.

## Phase-by-phase history

- **Phase 0–1**: environment bring-up; real FR3 kinematics + MoveIt 2
  planning + MuJoCo execution, no governor. Proved the base pipeline.
- **Phase 2**: B2 as a real `LocalConstraintSolverInterface` plugin
  (current-instant OSQP torque projection). Verified genuinely different
  behavior between within-limits and artificially-over-limit goals.
- **Phase 3a**: B3's certificate + Level 0/4 only. New
  `HorizonTrajectoryOperator` (needed because stock `SimpleSampler` only
  ever exposes one waypoint, not a horizon). `fr3_dynamics` extracted as a
  shared package so B2 and B3 don't duplicate dynamics math.
- **Phase 3b**: Level 1 (route-level retiming). New shared
  `torque_margin_certificate.{hpp,cpp}` (extracted `B3ConstraintSolver`'s
  margin math so the route-level search reuses it) +
  `route_retime_search.{hpp,cpp}`.
- **Phase 3c**: Level 2 (reshape), online + route-level. New
  `reshape_qp.{hpp,cpp}`, a much larger sparse OSQP QP than B2's
  single-step one (`3 × num_joints × n` variables).
- **Phase 3d (next, not started)**: Level 3 (reroute + candidate-route
  support). See "Next steps" below.

## Real bugs found and fixed along the way (worth knowing before you touch this code)

1. **macOS `@rpath` dlopen failure under `ros2 launch`** (Phase 3a). Pluginlib's
   `dlopen()` inside the running `component_container_mt` process couldn't
   resolve sibling-package `.dylib`s via `DYLD_LIBRARY_PATH`, even with a
   correctly-sourced overlay — affected `fr3_dynamics` AND an unrelated,
   pre-existing MoveIt plugin (`ReplanInvalidatedTrajectory` →
   `libsingle_plan_execution_plugin.dylib`, same directory). Root cause
   never fully nailed down (likely a `ros2 launch` composable-node-loading
   quirk specific to this RoboStack/macOS setup); fixed environment-
   independently with an explicit `@loader_path`-based
   `CMAKE_INSTALL_RPATH` in every affected `CMakeLists.txt`. **If you add
   a new package that depends on another package's shared library, you
   need this too** — copy the pattern from any existing `CMakeLists.txt`
   here (search for `CMAKE_INSTALL_RPATH`).
2. **Duplicate `declare_parameter` across plugins sharing a node.** Both
   `HorizonTrajectoryOperator` and `B3ConstraintSolver` load overlapping
   `b3.*` params into the *same* node (MoveIt loads both plugins into
   `local_planner_component`). Whichever initializes first (currently
   `HorizonTrajectoryOperator`) must declare; the other must read via
   `has_parameter`/`get_parameter` instead of a second `declare_parameter`,
   or it throws `ParameterAlreadyDeclaredException`. **Every new shared
   `b3.*` param needs this guard on both sides** — grep for
   `has_parameter("b3.dt")` for the pattern.
3. **`getStateAtDurationFromStart` writes into an already-allocated
   `RobotStatePtr`**, it does not allocate one. Passing a default-null
   pointer segfaults inside `RobotState::interpolate` (found via a macOS
   crash report, not guesswork).
4. **`HorizonTrajectoryOperator::addTrajectorySegment` must call
   `time_parametrization_.computeTimeStamps`** (like stock `SimpleSampler`
   does) — without it the buffered trajectory has no real duration and the
   local planner declares "done" almost instantly without the robot
   moving.
5. **The reshape QP's hard constraint only targets `|tau| ≤ tau_max -
   delta_tau`, not `m_safe`** (true of the Python reference too, not
   introduced here). Reshape reliably improves a genuine hard-constraint
   violation but won't always fully restore the separate `m_safe` safety
   buffer in one shot — this is expected, not a bug; Level 4 is the
   correct, always-available fallback when it doesn't.

## Known environmental gaps (disclosed, not hidden)

- **`mujoco_ros2_control` has a spontaneous crash window**, roughly 5–25s
  after launch, cause unknown (suspected GLFW/macOS window-server
  instability from repeated relaunches in one session — not caused by any
  patch here, confirmed via isolation testing). Mitigation: always poll
  for both the plugin-initialized log line AND the process being alive
  before sending a goal; don't idle between launch and verification.
- **No SCS solver in this environment** (checked directly — only
  `osqp`/`OsqpEigen` are present). The Python reference falls back from
  OSQP to SCS for the larger whole-route reshape QP; this port is
  OSQP-only. When OSQP can't converge on a big problem, the code reports
  it and falls through cleanly (matches the "retime exhausted" pattern) —
  it does not silently produce a wrong answer.
- **`m_safe = 2.0` N·m** (paper's own default, calibrated on a
  reduced-order 3-DOF arm) has never been validated as well- or
  poorly-calibrated at real FR3 scale. Not blocking, but flagged since
  Phase 3a.

## How to build and run

```bash
# Activate environment (RoboStack ROS 2 Humble in a conda env)
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh
conda activate ros_env
cd /Users/yycao/ai_learn/replan/ros2_ws
source install/setup.zsh

# Build everything touched by this platform
colcon build --packages-select fr3_dynamics fr3_b2_local_planner \
  fr3_b3_local_planner fr3_mujoco_bringup

# ALWAYS use a fresh ROS_DOMAIN_ID and kill stragglers before relaunching —
# DDS transient-local messages leak across launches otherwise.
pkill -9 -f "hybrid_planning|component_container|mujoco_ros2_control|robot_state_publisher|ros2_control_node|ros2 launch"
export ROS_DOMAIN_ID=<unused number>
ros2 launch fr3_mujoco_bringup fr3_b3_demo.launch.py   # or fr3_b2_demo.launch.py

# Poll for readiness before sending a goal (see "known gaps" above):
#   grep -q "B3ConstraintSolver initialized" <launch log> && pgrep -f mujoco_ros2_control
```

Test-config env vars (both launch files support these, default to the real values):
- `FR3_B2_TORQUE_LIMITS_YAML` — e.g. `config/b2_torque_limits_test_low.yaml`
- `FR3_B3_PARAMS_YAML` — e.g. `config/b3_params_test_low.yaml`,
  `config/b3_params_retime_test.yaml`, `config/b3_params_reshape_test.yaml`
  (see `fr3_b3_local_planner/config/` for what each exercises)

Debug instrumentation: set `B3_DEBUG_HORIZON=1` to get a per-waypoint
margin dump (`RCLCPP_INFO`, tagged `[horizon]`/`[whole-route]`/
`[online-reshape]`) — this is how every Phase 3b/3c test scenario was
actually tuned against real dynamics numbers instead of guessed.

Test scripts live in `scripts/`:
- `scripts/test_fr3_hybrid_planning.py` — small within-limits goal, the
  standard "zero interventions" regression check.
- `scripts/test_fr3_large_move.py` — large/fast goal, used to tune every
  Phase 3b/3c verification scenario against real measured dynamics.
- `scripts/sample_progress.py` — sends the large move and samples
  `/joint_states` at a fixed elapsed wall-clock time (bounded window, so
  it doesn't need to wait for full completion or risk the crash window
  below); used to compare execution progress between runs (e.g. nominal
  vs. Level-1-retimed).

Run any of them with the conda env's own interpreter (plain `python3` on
`PATH` resolves to the wrong one on this machine):
```bash
/opt/homebrew/Caskroom/miniconda/base/envs/ros_env/bin/python3 scripts/test_fr3_hybrid_planning.py
```

## Next steps

1. **Phase 3d: Level 3 (reroute)**. Needs candidate-route support that
   doesn't exist anywhere in this port yet (the Python reference's
   `plan_route` only *selects* between caller-supplied candidates — it
   doesn't generate them). Likely needs a hand-authored via-point
   alternate route (mirroring `code/trajectory.py::ViaPointTrajectory`),
   not OMPL multi-query support. Scope this the same way 3a–3c were
   scoped: read `plan_route`'s Level-3 branch and `ViaPointTrajectory`
   first, then write a plan file before touching code.
2. **Phase 4: port the experiment suite + metrics** onto ROS-bag-recorded
   data — this is where the platform stops being infrastructure and
   starts producing numbers that could go in the paper.
3. **Phase 5: run, verify, fold real numbers into the paper.**
4. **Sanity-check `m_safe = 2.0`** at real FR3 scale once there's a
   concrete experiment to check it against (see "Known environmental
   gaps").
5. This repo's commits are **local-only, never pushed** (per explicit
   instruction) — decide at some point whether/where this should go
   (a real remote, or fold into the main `replan` paper repo's scope
   despite its current `.gitignore` exclusion).
