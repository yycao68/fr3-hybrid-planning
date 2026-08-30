"""Phase 3a B3 verification: MoveIt 2 Hybrid Planning with B3's own
HorizonTrajectoryOperator + B3ConstraintSolver plugins
(fr3_b3_local_planner) in place of B2's SimpleSampler/B2ConstraintSolver,
against real FR3 kinematics + MuJoCo execution. Same shape as
fr3_b2_demo.launch.py, with local_planner_b3.yaml + B3's own certificate
params (b3_params.yaml) added to the local_planner ComposableNode.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    bringup_share = get_package_share_directory('fr3_mujoco_bringup')
    franka_description_share = get_package_share_directory('franka_description')

    # Phase 4b: end-effector payload mass (kg), read once here and used for
    # BOTH the URDF (so fr3_dynamics/the certificate sees it) and the MJCF
    # (so MuJoCo's own simulated physics carries it) -- see
    # fr3_mujoco.urdf.xacro's own comment for why both are needed.
    payload_mass_str = os.environ.get('FR3_PAYLOAD_MASS_KG', '0.0')

    # Phase 4c: external end-effector force schedule, read once here and
    # used for BOTH the MuJoCo injection node (force_mode/force_* params,
    # unprefixed) and B3's own predictive handling (b3.force_* params,
    # node-local prefix convention, not a ROS namespace) -- see
    # fr3_dynamics/force_schedule.hpp for what each field means.
    # FR3_FORCE_MODE="" (default) disables it entirely.
    # FR3_FORCE_KNOWN_AT_PLAN_TIME gates ROUTE-level (Level 1/2/3) force
    # awareness only -- the ONLINE path (B3ConstraintSolver) always
    # predicts forward regardless, matching code/baselines.py::policy_b3's
    # own always-sample-the-horizon behavior. "false" (default, Exp3): the
    # route-level search stays force-blind even though the online path
    # sees it. "true" (Exp4): route-level search sees it too.
    force_mode = os.environ.get('FR3_FORCE_MODE', '')
    force_t_onset = os.environ.get('FR3_FORCE_T_ONSET', '0.0')
    force_ramp_duration = os.environ.get('FR3_FORCE_RAMP_DURATION', '1.0')
    force_fx = os.environ.get('FR3_FORCE_FX', '0.0')
    force_fy = os.environ.get('FR3_FORCE_FY', '0.0')
    force_fz = os.environ.get('FR3_FORCE_FZ', '0.0')
    force_contact_z = os.environ.get('FR3_FORCE_CONTACT_Z', '0.0')
    force_k_contact = os.environ.get('FR3_FORCE_K_CONTACT', '0.0')
    force_known_at_plan_time = os.environ.get('FR3_FORCE_KNOWN_AT_PLAN_TIME', 'false').strip().lower() == 'true'
    b3_force_params = {
        'b3.force_mode': force_mode,
        'b3.force_t_onset': float(force_t_onset),
        'b3.force_ramp_duration': float(force_ramp_duration),
        'b3.force_fx': float(force_fx),
        'b3.force_fy': float(force_fy),
        'b3.force_fz': float(force_fz),
        'b3.force_contact_z': float(force_contact_z),
        'b3.force_k_contact': float(force_k_contact),
        'b3.force_known_at_plan_time': force_known_at_plan_time,
    }

    xacro_file = os.path.join(bringup_share, 'urdf', 'fr3_mujoco.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file, ' hand:=false',
         ' payload_mass:=', payload_mass_str])
    robot_description = {
        'robot_description': ParameterValue(robot_description_config, value_type=str)
    }

    srdf_file = os.path.join(
        franka_description_share, 'robots', 'fr3', 'fr3.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_file, ' hand:=false'])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_config, value_type=str)
    }

    kinematics_yaml = load_yaml('fr3_mujoco_bringup', 'config/kinematics.yaml')

    planning_pipelines_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                 'default_planner_request_adapters/ResolveConstraintFrames '
                                 'default_planner_request_adapters/FixWorkspaceBounds '
                                 'default_planner_request_adapters/FixStartStateBounds '
                                 'default_planner_request_adapters/FixStartStateCollision '
                                 'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('fr3_mujoco_bringup', 'config/ompl_planning.yaml')
    planning_pipelines_config['ompl'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'fr3_mujoco_bringup', 'config/fr3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                      '/MoveItSimpleControllerManager',
    }

    common_hybrid_planning_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/common_hybrid_planning_params.yaml')
    # Phase 4c-fix: deterministic trajectory replay (see
    # fr3_replay_global_planner's own header comment for why) -- default
    # (both env vars unset) leaves real OMPL planning unchanged;
    # exp2/exp3's sweeps set both so every cell/baseline replays one
    # captured trajectory instead of re-planning.
    global_planner_yaml = os.environ.get(
        'FR3_GLOBAL_PLANNER_YAML', 'config/hybrid_planning/global_planner.yaml')
    global_planner_param = load_yaml('fr3_mujoco_bringup', global_planner_yaml)
    replay_trajectory_path = os.environ.get('FR3_REPLAY_TRAJECTORY_PATH', '')
    if replay_trajectory_path:
        global_planner_param['replay_trajectory_path'] = replay_trajectory_path
    local_planner_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/local_planner_b3.yaml')
    # Root-cause fix (Phase 4c-fix determinism investigation): the local
    # trajectory operator's own re-time step (computeTimeStamps) defaults
    # to full URDF joint-limit speed regardless of the goal's own
    # max_velocity_scaling_factor/max_acceleration_scaling_factor --
    # moveit_msgs/MotionPlanResponse (all LocalPlannerComponent ever sees)
    # doesn't carry those fields, so they were never reachable here. These
    # default to 1.0 (a true no-op, matching every pre-existing baseline);
    # exp3_interaction_force.py sets them explicitly to match its own
    # goal's requested scale.
    local_scaling_param = {
        'local_velocity_scaling': float(os.environ.get('FR3_LOCAL_VEL_SCALE', '1.0')),
        'local_acceleration_scaling': float(os.environ.get('FR3_LOCAL_ACCEL_SCALE', '1.0')),
    }
    hybrid_planning_manager_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/hybrid_planning_manager.yaml')
    # FR3_B3_PARAMS_YAML lets the Phase 3a verification pass select an
    # artificially-tuned certificate config (e.g. a scaled-down tau_max) to
    # exercise B3's Level 4 intervention branch, without duplicating this
    # whole launch file. Defaults to the real-FR3-scale config.
    b3_params_file = os.environ.get(
        'FR3_B3_PARAMS_YAML', 'config/b3_params.yaml')
    b3_params_param = load_yaml('fr3_b3_local_planner', b3_params_file)

    hybrid_planning_container = ComposableNodeContainer(
        name='hybrid_planning_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_hybrid_planning',
                plugin='moveit::hybrid_planning::GlobalPlannerComponent',
                name='global_planner',
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_pipelines_config,
                    moveit_controllers,
                    {'use_sim_time': True},
                ],
            ),
            ComposableNode(
                package='moveit_hybrid_planning',
                plugin='moveit::hybrid_planning::LocalPlannerComponent',
                name='local_planner',
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    local_scaling_param,
                    b3_params_param,
                    b3_force_params,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    {'use_sim_time': True},
                ],
            ),
            ComposableNode(
                package='moveit_hybrid_planning',
                plugin='moveit::hybrid_planning::HybridPlanningManager',
                name='hybrid_planning_manager',
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                    {'use_sim_time': True},
                ],
            ),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Goal-execution-fragility oscillation fix: FR3_ARM_CONTROLLER_YAML lets
    # fr3_gravity_ff_controller be selected instead of the stock
    # joint_trajectory_controller without touching this launch file --
    # default is the exact prior filename, a true no-op.
    controller_config_filename = os.environ.get(
        'FR3_ARM_CONTROLLER_YAML', 'fr3_ros_controllers.yaml')
    controller_config_file = os.path.join(
        bringup_share, 'config', controller_config_filename)
    # Joint7-stall investigation (see ros2_ws/src/README.md's
    # goal-execution-fragility writeup, damping/inertial-coupling
    # isolation): FR3_MUJOCO_MODEL_FILENAME lets a modified MJCF be
    # selected without touching this launch file's own default -- unset
    # means the real, unmodified fr3.xml, a true no-op.
    mujoco_model_filename = os.environ.get('FR3_MUJOCO_MODEL_FILENAME', 'fr3.xml')
    mujoco_model_path = os.path.join(bringup_share, 'mujoco_models', mujoco_model_filename)
    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': mujoco_model_path},
            {'payload_mass_kg': float(payload_mass_str)},
            {'force_mode': force_mode},
            {'force_t_onset': float(force_t_onset)},
            {'force_ramp_duration': float(force_ramp_duration)},
            {'force_fx': float(force_fx)},
            {'force_fy': float(force_fy)},
            {'force_fz': float(force_fz)},
            {'force_contact_z': float(force_contact_z)},
            {'force_k_contact': float(force_k_contact)},
        ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_fr3_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fr3_arm_controller'],
        output='screen',
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_fr3_arm_controller],
            )
        ),
        node_mujoco_ros2_control,
        robot_state_publisher,
        hybrid_planning_container,
    ])
