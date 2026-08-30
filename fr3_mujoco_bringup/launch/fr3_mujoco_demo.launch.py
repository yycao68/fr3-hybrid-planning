"""Phase 1 bringup launch: real FR3 kinematics + MoveIt 2 planning +
MuJoCo-simulated execution via mujoco_ros2_control. No governor/plugin logic.

Modeled on two known-working references from this environment:
  - mujoco_ros2_control_demos/launch/cart_example_effort.launch.py for the
    mujoco_ros2_control node + controller-loading event-handler chain.
  - franka_ros2/franka_fr3_moveit_config/launch/moveit.launch.py (fetched
    from upstream for this port) for the move_group/RViz parameter shapes,
    adapted to source robot_description/robot_description_semantic from our
    own xacro + franka_description's fr3.srdf.xacro instead of
    franka_bringup (which requires franka_hardware/libfranka).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
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

    xacro_file = os.path.join(bringup_share, 'urdf', 'fr3_mujoco.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file, ' hand:=false'])
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

    ompl_planning_pipeline_config = {
        'move_group': {
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
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'fr3_mujoco_bringup', 'config/fr3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                      '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    rviz_config = os.path.join(bringup_share, 'rviz', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # mujoco_ros2_control publishes /clock as the sim time source (it runs
    # faster than wall-clock); every other node above sets use_sim_time so
    # its notion of "now" tracks /clock too -- without it, move_group treats
    # /joint_states messages (stamped with sim time) as stale against its
    # own wall-clock "now" and execution fails with couldn't receive full
    # current joint state.
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
        move_group_node,
        rviz_node,
    ])
