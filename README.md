# FR3 Hybrid Planning platform

Real Franka FR3 kinematics + MoveIt 2 Hybrid Planning + MuJoCo execution, built as the experimental platform for the "Predictive Physical Realizability" paper (`../predictive_realizability_paper_draft.md`, one level up — this `ros2_ws/` tree is `.gitignore`'d from that paper repo on purpose: it's engineering infrastructure, not paper content, so it has its own git history in *this* directory).

The paper's core claim is a certificate-driven, predictive local planner (**B3**) that catches a future torque violation before it happens, versus a reactive baseline (**B2**) that only checks the current instant. This platform ports both, plus the reduced-order Python reference's Level 0-4 response hierarchy, into real MoveIt 2 Hybrid Planning C++ plugins running against MuJoCo-simulated FR3 dynamics — not a toy model.

## Status: Phases 0–3d, 4a, 4b done. Phase 4c: Exp3 (unanticipated disturbance) done with a real positive result; Exp4 (known contact transition) blocked on a deeply-diagnosed but unresolved platform execution issue (see "Known environmental gaps").

## Architecture

```
fr3_mujoco_bringup/     Bring-up: URDF/xacro, MoveIt config, MuJoCo model,
                        launch files, controller config. No governor logic
                        of its own — wires everything else together. Both
                        the URDF and the MJCF carry a matching "payload"
                        link/body (Phase 4b) at fr3_link8's offset, mass
                        set at launch time from FR3_PAYLOAD_MASS_KG — see
                        "Real bugs found and fixed" below for why BOTH
                        models need it, not just the simulated one.
fr3_dynamics/           Shared KDL-based inverse dynamics (mass matrix,
                        Coriolis+gravity, KDL<->MoveIt joint index
                        mapping). Used by B2 and B3 so they share ONE
                        dynamics code path (paper's B2-vs-B3 fairness
                        principle) — this is load-bearing, not incidental.
                        Also has a Jacobian + FK solver and a shared
                        ForceSchedule struct (Phase 4c) for external
                        end-effector force handling (ramp or
                        position-triggered spring).
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
via_point_trajectory.{hpp,cpp}        Level 3: two-segment quintic
                                       stop-and-go candidate route
                                       (q0->q_via->qf), horizon_trajectory_
                                       operator_plugin only.
horizon_trajectory_operator.cpp       TrajectoryOperatorInterface plugin:
                                       exposes a real receding horizon
                                       (stock SimpleSampler only exposes
                                       one waypoint) + owns the ROUTE-LEVEL
                                       Level 1/2/3 decision (once per new
                                       trajectory).
b3_constraint_solver.cpp              LocalConstraintSolverInterface
                                       plugin: certificate check + ONLINE
                                       Level 0/2/4 decision (every cycle).
```

Third-party trees (`franka_description`, `moveit_hybrid_planning`, `mujoco_ros2_control`, `mujoco_menagerie_sparse`) are `.gitignore`'d here — each is its own clone with its own upstream history. Two of them carry local-only patches (not pushed, no push access to upstream):
- `moveit_hybrid_planning`: `@loader_path` install-RPATH fix (see below).
- `mujoco_ros2_control`: real-time pacing patch (sleeps off sim-vs-wall-clock drift each outer-loop iteration).

## The Level 0–4 hierarchy (paper Sec. V-B, `code/local_planner.py`)

| Level | What | Where | Cadence |
|---|---|---|---|
| 0 | Pass nominal through | both plugins | — |
| 1 | Retime (slow down by λ∈[1,λ_max]) | `HorizonTrajectoryOperator` | once per new route |
| 2 | Reshape (QP over qddot) | both, route- and cycle-level | route: once/route; online: every cycle |
| 3 | Reroute (via-point candidate, caller-configured) | `HorizonTrajectoryOperator` | once per new route, only if 1 and 2 both fail |
| 4 | Sticky brake | `B3ConstraintSolver` | every cycle, once triggered, forever |

Ordering per route: **retime → reshape → reroute**, tried only if the previous one didn't fix things; reroute itself re-runs retime then reshape on the alternate candidate (sub-levels 0/1/2) before giving up. Online, per cycle: **reshape → brake**. This exactly mirrors `code/local_planner.py::plan_route` / `online_step`'s own cascade. Level 3's candidate is **caller-configured** (`b3.via_point_offset.*`, all-zero by default = no candidate = true no-op), not searched for or generated — matches the Python reference's own explicit scope ("certificate-guided selection among caller-supplied candidates, not a general replanner").

## Phase-by-phase history

- **Phase 0–1**: environment bring-up; real FR3 kinematics + MoveIt 2 planning + MuJoCo execution, no governor. Proved the base pipeline.
- **Phase 2**: B2 as a real `LocalConstraintSolverInterface` plugin (current-instant OSQP torque projection). Verified genuinely different behavior between within-limits and artificially-over-limit goals.
- **Phase 3a**: B3's certificate + Level 0/4 only. New `HorizonTrajectoryOperator` (needed because stock `SimpleSampler` only ever exposes one waypoint, not a horizon). `fr3_dynamics` extracted as a shared package so B2 and B3 don't duplicate dynamics math.
- **Phase 3b**: Level 1 (route-level retiming). New shared `torque_margin_certificate.{hpp,cpp}` (extracted `B3ConstraintSolver`'s margin math so the route-level search reuses it) + `route_retime_search.{hpp,cpp}`.
- **Phase 3c**: Level 2 (reshape), online + route-level. New `reshape_qp.{hpp,cpp}`, a much larger sparse OSQP QP than B2's single-step one (`3 × num_joints × n` variables).
- **Phase 3d**: Level 3 (reroute). New `via_point_trajectory.{hpp,cpp}` (two-segment quintic stop-and-go candidate, ported from `code/trajectory.py::ViaPointTrajectory`). Reused Level 1/2's own machinery unchanged to evaluate/fix the candidate — genuinely small new code surface for what the hierarchy diagram makes look like a big phase. Verified: the mechanism (candidate construction, margin evaluation, retime/reshape cascade on the alternate, correct fallthrough) works correctly in every trial; the *specific* via-point offsets tried (6 trials, several directions/magnitudes/joint combinations) consistently made a wrist-joint-tight test scenario worse, not better — the stop-and-go candidate's own extra peak-acceleration demand outweighed any gravity-loading benefit found, and this held even after retiming the candidate up to `lam_max`. See `fr3_b3_local_planner/config/b3_params_reroute_test.yaml`'s header for the full finding.
- **Phase 4a**: observability + metrics harness, the first slice of "port the experiment suite" (a large, multi-experiment body of work in the Python reference — split the same way Phase 3 was split into 3a–3d). Added `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, no new interface package needed) published every cycle by both `B2ConstraintSolver` and `B3ConstraintSolver` — the per-cycle state that had no log line before (Level 0 pass-through is silent). New `scripts/run_experiment.py` (formalizes the manual pkill → fresh domain ID → launch → poll-ready → send-goal dance every prior phase did by hand into a reusable harness that also drives `ros2 bag record`) and `scripts/compute_metrics.py` (reads the bag via `rosbag2_py`, computes a deliberately-scoped subset of `code/metrics.py::RunMetrics`'s fields). Verified: B1 (stock)/B2/B3 all succeed on the standard within-limits goal with near-identical final error and zero interventions for B2/B3 — the direct analog of the Python reference's own Exp 1 "no regression" check. Along the way, found the real controller has a genuine steady-state joint-space tracking residual (~0.045–0.05 rad L2, confirmed stable across a 1s settle window, not a transient) that the `HybridPlanner` action's own "SUCCESS" doesn't reflect — `compute_metrics.py`'s success tolerance is set from this measured floor, not an idealized number.
- **Phase 4b**: payload sweep (Exp 2 analog). New end-effector `"payload"` body (MJCF, `fr3.xml`) + matching `"payload"` link/`payload_joint` fixed joint (URDF, `fr3_mujoco.urdf.xacro`), both at `fr3_link8`'s pose, mass threaded from one new `FR3_PAYLOAD_MASS_KG` env var into both models from the same launch-time value (see "Real bugs found and fixed" for why both are required, not just the simulated one). `run_experiment.py`/`compute_metrics.py` refactored into reusable `run_one()`/`compute()` library functions (still work as before from the CLI) plus a `"large"` goal choice, so the new `scripts/exp2_payload_sweep.py` can call them directly across many runs instead of shelling out per-cell. Verified end-to-end: **B3's certificate first triggers an online intervention at 6kg payload, 4kg before B2's own self-reported intervention at 10kg** — a genuine, monotonic "certificate lead" (`min_margin` trends 9.65 → 5.81 → 3.21 → 1.95 → 1.13 → -12.4 → -24.2 N·m as payload rises from 0 to 10kg). B1 could not be used as a comparison baseline on this goal at all — see "Known environmental gaps."
- **Phase 4c**: external end-effector force injection (Exp 3/4 analogs). `fr3_dynamics` gained a Jacobian (`computeJacobian`), an FK solver (`computeTipPosition`), and a shared `ForceSchedule` struct (`force_schedule.hpp`, ramp or position-triggered spring), threaded through every B2/B3 call site (`b2_constraint_solver.cpp`'s reactive check; `torque_margin_certificate.cpp`/`reshape_qp.cpp`/`route_retime_search.cpp`'s online AND route-level paths, gated by a new `b3.force_known_at_plan_time` flag that is the entire Exp3-vs-Exp4 code-path difference) — matches `dynamics.py::Arm.required_torque`'s own `tau - J(q)^T@F_ext` mechanism line for line. MuJoCo injection lives in `mujoco_ros2_control.cpp`'s `update()` loop, triggered by a shared one-shot `/fr3_force_injection/start` topic (`run_experiment.py`'s `run_one()` publishes it at goal-*acceptance* time, not launch time — see "Real bugs found and fixed"). **Exp3 (unanticipated ramp disturbance, `force_known_at_plan_time=false`)**: done, with a real, reproducible, positive result — **B3's certificate detects the oncoming violation ~0.36–0.38s before the ground-truth failure time; B2 never detects it at all**, confirmed across two independent runs (`scripts/exp3_interaction_force.py`). Ground truth is computed by `scripts/ground_truth.py`, reusing a debug-only `"[whole-route-extended]"` certificate probe in `horizon_trajectory_operator.cpp` (a held-terminal-state extension of the real route, needed because the certificate's own online receding horizon evaluates well past a short route's own completion — this probe does NOT feed the real Level 1/2/3 decision). **Exp4 (known-in-advance position-triggered contact, `force_known_at_plan_time=true`)**: blocked — see "Known environmental gaps" for the goal-execution-fragility platform issue found while designing its goal, not a Phase 4c bug.

## Real bugs found and fixed along the way (worth knowing before you touch this code)

1. **macOS `@rpath` dlopen failure under `ros2 launch`** (Phase 3a). Pluginlib's `dlopen()` inside the running `component_container_mt` process couldn't resolve sibling-package `.dylib`s via `DYLD_LIBRARY_PATH`, even with a correctly-sourced overlay — affected `fr3_dynamics` AND an unrelated, pre-existing MoveIt plugin (`ReplanInvalidatedTrajectory` → `libsingle_plan_execution_plugin.dylib`, same directory). Root cause never fully nailed down (likely a `ros2 launch` composable-node-loading quirk specific to this RoboStack/macOS setup); fixed environment- independently with an explicit `@loader_path`-based `CMAKE_INSTALL_RPATH` in every affected `CMakeLists.txt`. **If you add a new package that depends on another package's shared library, you need this too** — copy the pattern from any existing `CMakeLists.txt` here (search for `CMAKE_INSTALL_RPATH`).
2. **Duplicate `declare_parameter` across plugins sharing a node.** Both `HorizonTrajectoryOperator` and `B3ConstraintSolver` load overlapping `b3.*` params into the *same* node (MoveIt loads both plugins into `local_planner_component`). Whichever initializes first (currently `HorizonTrajectoryOperator`) must declare; the other must read via `has_parameter`/`get_parameter` instead of a second `declare_parameter`, or it throws `ParameterAlreadyDeclaredException`. **Every new shared `b3.*` param needs this guard on both sides** — grep for `has_parameter("b3.dt")` for the pattern.
3. **`getStateAtDurationFromStart` writes into an already-allocated `RobotStatePtr`**, it does not allocate one. Passing a default-null pointer segfaults inside `RobotState::interpolate` (found via a macOS crash report, not guesswork).
4. **`HorizonTrajectoryOperator::addTrajectorySegment` must call `time_parametrization_.computeTimeStamps`** (like stock `SimpleSampler` does) — without it the buffered trajectory has no real duration and the local planner declares "done" almost instantly without the robot moving.
5. **The reshape QP's hard constraint only targets `|tau| ≤ tau_max - delta_tau`, not `m_safe`** (true of the Python reference too, not introduced here). Reshape reliably improves a genuine hard-constraint violation but won't always fully restore the separate `m_safe` safety buffer in one shot — this is expected, not a bug; Level 4 is the correct, always-available fallback when it doesn't.
6. **B2/B3's certificate is entirely blind to a payload added only to the MJCF** (Phase 4b). `fr3_dynamics` builds its KDL chain from the URDF (`kdl_parser`), completely independent of MuJoCo's own MJCF — adding a payload body to `fr3.xml` alone changes only the *simulated* physics, not what the certificate computes at all (confirmed via `B3_DEBUG_HORIZON`: byte-identical margins at 5kg vs. 0kg payload before the fix). Fixed by also adding a matching `"payload"` link to the URDF (`fr3_mujoco.urdf.xacro`, child of `fr3_link8` via a fixed joint) and switching B2/B3's `tip_link` config from `"fr3_link8"` to `"payload"` in all 8 relevant config YAMLs, so the mass genuinely enters the KDL chain. Re-verified after the fix: margin at step0 dropped from 9.92 (0kg) to 3.13 (5kg), confirming the certificate now sees it. **If you ever add another MJCF-only body expecting the certificate to react to it, it won't — the URDF is the only thing `fr3_dynamics` ever reads.**
7. **A leaked-daemon DDS hang during long sequential runs** (Phase 4b). `ros2 control load_controller` (used in every launch file here) auto-spawns a persistent per-`ROS_DOMAIN_ID` `ros2cli.daemon.daemonize` background process holding its own live rclpy node/DDS participant open for up to 2 hours of inactivity. Since `run_one()` picks a fresh random domain ID on every call, a long sequence of runs (e.g. a payload sweep) leaks one daemon per call; `pkill_stragglers()`'s pattern never matched them. Confirmed live: 25+ accumulated over a ~50-minute sweep and wedged the DDS transport solid (every UDP write failing, one cell hanging indefinitely with no error at all — it never even reached starting that cell's `ros2 launch`). Fixed by adding `daemonize` to `run_experiment.py`'s `PKILL_PATTERN`, so every `run_one()` call reaps all accumulated stragglers, not just its own. **Any new script that calls `run_one()` in a loop relies on this — don't bypass `pkill_stragglers()`.**
8. **`mujoco_ros2_control`'s own ROS subscriptions were never dispatched at all** (Phase 4c). `MujocoRos2Control::init()` only ever added `controller_manager_` to `cm_executor_` (its own `MultiThreadedExecutor`) — `node_` itself, the node any NEW subscription (e.g. the Phase 4c force-injection start signal) gets created on, was never spun by anything. Confirmed live: the callback simply never fired, silently leaving the SIMULATED force never applied even though B2/B3's own certificate (a separate process, correctly spun by ITS OWN container) reacted as if it had — a fully believable but wrong result if not caught. Fixed with one line, `cm_executor_->add_node(node_);`, reusing the already-running background executor thread rather than adding a new spin loop. **Any new subscription/timer added to `mujoco_ros2_control`'s own top-level node needs this — it is not spun by default.**
9. **The force-injection start trigger fired before `send_goal()`, not at goal acceptance** (Phase 4c). Publishing it right after `wait_ready()` (matching the original design) left several seconds of unpredictable global+local planning overhead for a multi-second force ramp to complete during, before any real execution ever began — confirmed live: a 0.5s ramp was already fully held by the time a single online `solve()` cycle ran. Fixed by adding an `on_accepted` hook to `send_goal()` that fires the trigger the moment the `HybridPlanner` action server accepts the goal (much closer to real execution start) — reusing `send_goal()`'s own already-active `rclpy` node rather than creating a second one (a nested `rclpy.init()` while already spinning is not safe).
10. **Three metrics/harness robustness bugs found by an independent external review of `scripts/` (Phase 4c), all confirmed real by re-reading the actual code before fixing, all fixed and re-verified against the established baseline** (`min_margin=9.6489` on the small goal, unchanged): (a) `compute_metrics.py` silently dropped any joint missing from the final `/joint_states` sample out of the L2 error sum instead of failing loudly — understates error, could produce a false `task_success`; now raises `RuntimeError` listing the missing joints. (b) `compute_metrics.py` assumed the *first* `/diagnostics` message's *first* status entry was the controller's own — fragile if any other diagnostic publisher ever appeared on the topic first; now explicitly searches every message for `"b2_constraint_solver"`/`"b3_constraint_solver"` by name. (c) `run_experiment.py`'s `run_one()` had no `try`/`finally` around the launch+bag+goal sequence — an exception inside `send_goal()` left `bag_proc`/`launch_proc` running until a *later* call's own `pkill_stragglers()` happened to clean them up; now wrapped so teardown always runs. **The same review also raised deeper, unresolved experimental-validity concerns not yet acted on** — most importantly, `exp2_payload_sweep.py` and `exp3_interaction_force.py` each independently trigger fresh (randomized, unseeded) OMPL planning per cell/baseline, so the paper's own "identical geometric trajectory, only X varies" framing for Exp2/Exp3 is not yet actually guaranteed by the code — see "Known environmental gaps."

## Known environmental gaps (disclosed, not hidden)

- **`mujoco_ros2_control` has a spontaneous crash window**, roughly 5–25s after launch, cause unknown (suspected GLFW/macOS window-server instability from repeated relaunches in one session — not caused by any patch here, confirmed via isolation testing). Mitigation: always poll for both the plugin-initialized log line AND the process being alive before sending a goal; don't idle between launch and verification.
- **No SCS solver in this environment** (checked directly — only `osqp`/`OsqpEigen` are present). The Python reference falls back from OSQP to SCS for the larger whole-route reshape QP; this port is OSQP-only. When OSQP can't converge on a big problem, the code reports it and falls through cleanly (matches the "retime exhausted" pattern) — it does not silently produce a wrong answer.
- **`m_safe = 2.0` N·m** (paper's own default, calibrated on a reduced-order 3-DOF arm) has never been validated as well- or poorly-calibrated at real FR3 scale. Not blocking, but flagged since Phase 3a.
- **B1 (stock ForwardTrajectory/SimpleSampler) cannot complete the "large" goal at all**, independent of payload (Phase 4b). Confirmed directly: at payload=0/3/5/7kg its joint_states show ~0.007 rad drift (sensor noise) for the *entire* 45s window — it never starts moving; at 10kg it moves substantially (1.78 rad) but still never reaches the target. B2 and B3 both execute this exact goal fine at payload=0, so the goal itself and the payload plumbing are sound — this is specific to B1's own stock local-planner pipeline on this particular goal, first surfaced in Phase 4b because it's the first time B1 was ever pointed at the large goal (Phase 4a's B1 check used the small goal). Root cause not investigated (out of scope — B1 was only needed here as a payload-sweep comparison point, not as something to fix); `exp2_payload_sweep.py` detects this case (B1 failing at the lowest payload swept) and prints a caveat instead of a misleading "certificate lead over B1" number.
- **`mujoco_ros2_control`'s macOS build needs `MUJOCO_DIR` set explicitly and `-DCMAKE_FIND_FRAMEWORK=LAST`** after any `--cmake-clean-cache` (Phase 4b, self-inflicted while adding the payload patch). Without the first, CMake can't find `find_package(mujoco)` at all; without the second, it resolves to the `mujoco.framework` bundle in the same `lib/` dir instead of the plain `.dylib` and fails to link (`ld: framework not found mujoco`). Only matters after a clean-cache rebuild of this one package — normal incremental builds are unaffected.
- **Goal-execution fragility for any single joint moved beyond ~0.13-0.15 rad from `"small"`'s own specific, previously-validated combination** (Phase 4c, found while designing Exp4's goal — needed real end-effector Z travel, which `"small"`'s own joint targets don't provide). This is the SAME underlying issue as Phase 4b's own "B1 can't complete `large`" finding above, now understood to be more general (affects B2 too, not just B1) and NOT joint2-specific (confirmed: joint1 alone at -0.15 rad, everything else at `"small"`'s baseline, fails identically). Deeply diagnosed (ten layers checked with direct evidence) but not root-caused:
  - Global planning (OMPL) succeeds cleanly — a valid 2-waypoint solution is found and hands off correctly.
  - B2's own torque-limit QP is not the cause — `intervened=false`, healthy ~10 N·m margin throughout.
  - Not a velocity/acceleration-scaling artifact — confirmed broken at scale 0.15, 0.5, AND 1.0.
  - `SimpleSampler`'s own waypoint-advancement source (read directly) looks structurally correct.
  - The local planner genuinely publishes a real, non-trivial target on `/<controller>/joint_trajectory` (confirmed via bag) — but `fr3_arm_controller`'s own `/controller_state` topic shows its internal `reference` never adopts it, staying within ~0.003 rad of the start position for the entire run, with topic/QoS wiring confirmed correct (1 publisher, 1 subscriber, matching QoS) and zero rejection/error messages even with `fr3_arm_controller`'s own logger elevated to DEBUG.
  - Further diagnosis would need `joint_trajectory_controller`'s own compiled internals (only headers are available locally, not the `.cpp` implementation) or a `FollowJointTrajectory`-action-level trace (ruled out as the actual mechanism: `LocalPlannerComponent` publishes a raw topic directly, confirmed from its own source, not an action goal).
  - **Practical implication**: any future goal design must stay close to `"small"`'s own validated joint values, or budget time to actually resolve this first — it blocks any goal needing meaningful task-space travel (Exp4's contact-spring experiment, in particular).
- **`exp2_payload_sweep.py`/`exp3_interaction_force.py` do not currently guarantee an identical geometric trajectory across cells** (found by an independent external review, Phase 4c, confirmed real by re-reading the code). Each cell/baseline calls `run_one()` → `send_goal()` → a fresh `MotionPlanRequest`, and OMPL's RRTConnect is randomized with no seed pinned anywhere in `ompl_planning.yaml` — so payload (Exp2) or the force schedule (Exp3) is not provably the *only* thing that changed between runs being compared, which is exactly what the paper's own "identical geometric trajectory, geometric path and time law held fixed" framing requires. `ground_truth.py`'s own `T_failure` (from a separate ground-truth run) and `exp3`'s `T_detection` (from separate B2/B3 runs) are therefore not guaranteed to refer to the same nominal path either, even though the reported `T_warning` numbers were independently reproducible run-to-run in the two live trials done this session. The robust fix is to plan once, save the resulting trajectory, and replay it (bypassing fresh OMPL calls) across every baseline/configuration in a sweep — real new engineering (most plausibly a trivial "replay" global-planner plugin, since a fixed RNG seed alone would reduce but not eliminate OMPL's own time-budget-dependent nondeterminism), not yet implemented. Separately, `ground_truth.py`'s "ground truth" is B3's own certificate model re-evaluated offline, not independent physics — this matches the Python reference's own `ground_truth_failure_time`/`Certificate.ground_truth_violation` methodology exactly (confirmed by re-reading it), so it's a faithful port, not a shortcut introduced here, but the paper should describe it as a model-consistent reference rather than an independent validation.

## How to build and run

```bash
# Activate environment (RoboStack ROS 2 Humble in a conda env)
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh
conda activate ros_env
cd /Users/yycao/ai_learn/replan/ros2_ws
source install/setup.zsh

# Build everything touched by this platform
colcon build --packages-select fr3_dynamics fr3_b2_local_planner \ fr3_b3_local_planner fr3_mujoco_bringup

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
- `FR3_B3_PARAMS_YAML` — e.g. `config/b3_params_test_low.yaml`, `config/b3_params_retime_test.yaml`, `config/b3_params_reshape_test.yaml`, `config/b3_params_reroute_test.yaml` (see `fr3_b3_local_planner/config/` for what each exercises)
- `FR3_PAYLOAD_MASS_KG` — end-effector payload in kg (Phase 4b), all three launch files support it, default `"0.0"`. Sets both the URDF and the MJCF's `"payload"` body/link so B2/B3's certificate and MuJoCo's simulated physics agree.
- `FR3_FORCE_MODE` (`""`/`"ramp"`/`"spring"`), `FR3_FORCE_T_ONSET`, `FR3_FORCE_RAMP_DURATION`, `FR3_FORCE_FX/FY/FZ`, `FR3_FORCE_CONTACT_Z`, `FR3_FORCE_K_CONTACT` (Phase 4c) — external end-effector force schedule, all three launch files support it, default disabled (`""`). Sets both the MuJoCo injection node's params and B2/B3's own `b2.force_*`/`b3.force_*` params from the same values. `FR3_FORCE_KNOWN_AT_PLAN_TIME` (B3's `fr3_b3_demo.launch.py` only) gates whether B3's route-level Level 1/2/3 search sees the schedule too (Exp4) or only the online path does (Exp3, the default).

Debug instrumentation: set `B3_DEBUG_HORIZON=1` to get a per-waypoint margin dump (`RCLCPP_INFO`, tagged `[horizon]`/`[whole-route]`/`[online-reshape]`/`[whole-route-extended]`, each line also carries the waypoint's own absolute schedule time `t=` and FK `ee_z=`) — this is how every Phase 3b/3c/3d/4c test scenario was actually tuned against real dynamics numbers instead of guessed.

Test scripts live in `scripts/`:
- `scripts/test_fr3_hybrid_planning.py` — small within-limits goal, the standard "zero interventions" regression check.
- `scripts/test_fr3_large_move.py` — large/fast goal, used to tune every Phase 3b/3c/3d verification scenario against real measured dynamics.
- `scripts/sample_progress.py` — sends the large move and samples `/joint_states` at a fixed elapsed wall-clock time (bounded window, so it doesn't need to wait for full completion or risk the crash window below); used to compare execution progress between runs (e.g. nominal vs. Level-1-retimed).
- `scripts/run_experiment.py` — Phase 4a's harness: launch + poll-ready + `ros2 bag record` (`/joint_states`, `/diagnostics`, `/rosout`) + send the standard goal + teardown, in one command. `python3 run_experiment.py --launch-file fr3_b3_demo.launch.py --bag-dir /tmp/some_run`. Add `--goal large` and/or `--payload-mass-kg 5.0` for Phase 4b-style runs. `run_one(...)` is the reusable entry point other scripts import (raises `RuntimeError` rather than exiting).
- `scripts/compute_metrics.py` — reads a bag `run_experiment.py` produced and prints `final_pos_error_rad`/`task_success`/`min_margin`/`online_intervention_count`/route-level event counts. `python3 compute_metrics.py /tmp/some_run [--goal large]`. `compute(...)` is the reusable entry point (returns a result dict).
- `scripts/exp2_payload_sweep.py` — Phase 4b's payload sweep: runs B1/B2/B3 across a swept `FR3_PAYLOAD_MASS_KG` on the large goal, reports the payload at which each baseline first shows a violation vs. where B3's certificate first triggers ("certificate lead in kg"). `python3 exp2_payload_sweep.py [payload_kg ...]` (defaults to a coarse `[0, 3, 5, 6, 7, 8, 10]` sweep).
- `scripts/ground_truth.py` — Phase 4c: reads a `B3_DEBUG_HORIZON=1` launch log's `"[whole-route-extended]"` lines (a debug-only certificate probe, see `horizon_trajectory_operator.cpp`) to compute the ground-truth failure/contact time for the UNMODIFIED nominal route, matching `code/experiments/exp3_interaction_force.py::ground_truth_failure_time`'s own role. `ground_truth_failure_time(launch_log_path)`.
- `scripts/exp3_interaction_force.py` — Phase 4c Exp3: unanticipated ramp disturbance (`force_known_at_plan_time=false`), reports `T_warning = T_failure - T_detection` for B2/B3. `python3 exp3_interaction_force.py`. Real, reproducible result: B3 detects ~0.36-0.38s before the ground-truth failure; B2 never detects it.
- `scripts/exp4_contact_transition.py` — Phase 4c Exp4: known-in-advance position-triggered contact (`force_known_at_plan_time=true`), reports task success/route-level events per baseline. Written but **not currently usable** — blocked on the goal-execution-fragility issue in "Known environmental gaps" (no goal with meaningful end-effector Z travel executes reliably yet).

Run any of them with the conda env's own interpreter (plain `python3` on `PATH` resolves to the wrong one on this machine):
```bash
/opt/homebrew/Caskroom/miniconda/base/envs/ros_env/bin/python3 scripts/test_fr3_hybrid_planning.py
```

## Next steps

1. **Root-cause the goal-execution-fragility issue** (see "Known environmental gaps") — the highest-priority open item. It blocks Phase 4c's own Exp4, and likely any future goal needing meaningful task-space travel beyond `"small"`'s own specific validated joint combination (this is very probably the same root cause as Phase 4b's original "B1 can't complete `large`" finding, now shown to be more general). Ten diagnostic layers are already ruled out with direct evidence (see the finding's own writeup) — the next step is either `joint_trajectory_controller` source-level debugging (no local `.cpp` source currently available, only headers) or an experiment-design workaround that avoids the fragile region entirely.
2. **Finish Phase 4c: Exp4** (known-in-advance contact transition) once (1) is resolved — `scripts/exp4_contact_transition.py` and the spring-mode injection mechanism are already built and ready, just blocked on having a working goal.
3. **Phase 4d: flagship reroute + severity sweep + environment-conditioned reroute** (Exp 5/6/7 analogs), reusing Phase 3d's Level 3 + Phase 4b's payload + Phase 4c's force injection together.
4. **Phase 5: run everything, fold real numbers into the paper** — Phase 4c's Exp3 result (B3 detects ~0.36-0.38s before ground-truth failure, B2 never detects) is already real and ready to use.
5. **Optional, not blocking: find a via-point offset that actually recovers margin.** Phase 3d's own verification found 6 tried offsets all made a specific test scenario worse (see Phase 3d history above) — the mechanism is correct, but nobody has yet found a case where Level 3 demonstrably helps. Worth a fresh empirical pass with `B3_DEBUG_HORIZON` if a real experiment needs it, but not worth chasing for its own sake.
6. **Sanity-check `m_safe = 2.0`** at real FR3 scale once there's a concrete experiment to check it against (see "Known environmental gaps").
7. This repo's commits are **local-only, never pushed** (per explicit instruction) — decide at some point whether/where this should go (a real remote, or fold into the main `replan` paper repo's scope despite its current `.gitignore` exclusion).
