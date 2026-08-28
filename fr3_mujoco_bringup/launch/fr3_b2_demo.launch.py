"""Phase 2 B2 verification: MoveIt 2 Hybrid Planning with B2's own
LocalConstraintSolverInterface plugin (fr3_b2_local_planner/B2ConstraintSolver)
in place of the stock ForwardTrajectory, against real FR3 kinematics +
MuJoCo execution. Same shape as fr3_hybrid_planning_demo.launch.py (the
Phase 2 step 2 smoke test), with local_planner_b2.yaml + B2's own real
FR3 torque-limit params added to the local_planner ComposableNode.
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
    b2_share = get_package_share_directory('fr3_b2_local_planner')

    # Phase 4b: end-effector payload mass (kg), read once here and used for
    # BOTH the URDF (so fr3_dynamics/the certificate sees it) and the MJCF
    # (so MuJoCo's own simulated physics carries it) -- see
    # fr3_mujoco.urdf.xacro's own comment for why both are needed.
    payload_mass_str = os.environ.get('FR3_PAYLOAD_MASS_KG', '0.0')

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
    global_planner_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/global_planner.yaml')
    local_planner_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/local_planner_b2.yaml')
    hybrid_planning_manager_param = load_yaml(
        'fr3_mujoco_bringup', 'config/hybrid_planning/hybrid_planning_manager.yaml')
    # FR3_B2_TORQUE_LIMITS_YAML lets the Phase 2 verification pass select
    # the artificially-low test config (b2_torque_limits_test_low.yaml) to
    # exercise B2's intervention branch, without duplicating this whole
    # launch file. Defaults to the real FR3 limits.
    b2_torque_limits_file = os.environ.get(
        'FR3_B2_TORQUE_LIMITS_YAML', 'config/b2_torque_limits.yaml')
    b2_torque_limits_param = load_yaml('fr3_b2_local_planner', b2_torque_limits_file)

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
                    b2_torque_limits_param,
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

    controller_config_file = os.path.join(
        bringup_share, 'config', 'fr3_ros_controllers.yaml')
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
