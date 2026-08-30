"""Phase 2 smoke test: MoveIt 2 Hybrid Planning's stock plugins
(ForwardTrajectory local-constraint-solver + SimpleSampler trajectory
operator + ReplanInvalidatedTrajectory planner logic) against real FR3
kinematics + MuJoCo execution via mujoco_ros2_control.

Modeled directly on two known-working references from this environment:
  - moveit_hybrid_planning's own test/launch/hybrid_planning_common.py
    (generate_common_hybrid_launch_description) for the composable-node
    container shape (global_planner + local_planner + hybrid_planning_manager)
    -- confirmed working end-to-end against the stock Panda+FakeSystem demo
    in this session before this file was written.
  - fr3_mujoco_bringup/launch/fr3_mujoco_demo.launch.py for the
    mujoco_ros2_control + robot_state_publisher + controller-loading chain.

No custom plugin code exists yet -- this proves the Hybrid Planning
plugin-loading/action pipeline works against FR3+MuJoCo specifically,
before any B2 QP logic is written (Phase 2 plan, step 2).
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
    # used for the MuJoCo injection node's own force_mode/force_* params
    # (unprefixed). B1 has no force-aware code at all (stock plugins, by
    # design -- "B1 has no detection mechanism," code/experiments/
    # exp3_interaction_force.py's own docstring) -- this only affects the
    # SIMULATED physics B1 gets pushed around by, not any B1 decision.
    # FR3_FORCE_MODE="" (default) disables it entirely.
    force_mode = os.environ.get('FR3_FORCE_MODE', '')
    force_t_onset = os.environ.get('FR3_FORCE_T_ONSET', '0.0')
    force_ramp_duration = os.environ.get('FR3_FORCE_RAMP_DURATION', '1.0')
    force_fx = os.environ.get('FR3_FORCE_FX', '0.0')
    force_fy = os.environ.get('FR3_FORCE_FY', '0.0')
    force_fz = os.environ.get('FR3_FORCE_FZ', '0.0')
    force_contact_z = os.environ.get('FR3_FORCE_CONTACT_Z', '0.0')
    force_k_contact = os.environ.get('FR3_FORCE_K_CONTACT', '0.0')

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
        'fr3_mujoco_bringup', 'config/hybrid_planning/local_planner.yaml')
    # Root-cause fix (Phase 4c-fix determinism investigation): see
    # fr3_b3_demo.launch.py's own comment on local_scaling_param -- stock
    # SimpleSampler (B1's own trajectory operator) has the same gap, fixed
    # the same way. Defaults to 1.0/1.0, a true no-op.
    local_scaling_param = {
        'local_velocity_scaling': float(os.environ.get('FR3_LOCAL_VEL_SCALE', '1.0')),
        'local_acceleration_scaling': float(os.environ.get('FR3_LOCAL_ACCEL_SCALE', '1.0')),
    }
    hybrid_planning_manager_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/hybrid_planning_manager.yaml')

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

    # mujoco_ros2_control publishes /clock as the sim time source; see
    # fr3_mujoco_demo.launch.py for why every node above sets use_sim_time.
    # Goal-execution-fragility oscillation fix: FR3_ARM_CONTROLLER_YAML lets
    # fr3_gravity_ff_controller be selected instead of the stock
    # joint_trajectory_controller without touching this launch file --
    # default is the exact prior filename, a true no-op.
    controller_config_filename = os.environ.get(
        'FR3_ARM_CONTROLLER_YAML', 'fr3_ros_controllers.yaml')
    controller_config_file = os.path.join(
        bringup_share, 'config', controller_config_filename)
    mujoco_model_path = os.path.join(bringup_share, 'mujoco_models', 'fr3.xml')
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
