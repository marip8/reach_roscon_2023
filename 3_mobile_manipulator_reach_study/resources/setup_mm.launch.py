from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml


parameters = [
  {'name': 'robot_description_file',                'description': 'Path to the URDF/xacro file',                     'default': PathJoinSubstitution([FindPackageShare('reach_roscon_2023'), '3_mobile_manipulator_reach_study', 'resources', 'reach_study.xacro'])},
  {'name': 'robot_description_semantic_file',       'description': 'Path to the SRDF file',                           'default': PathJoinSubstitution([FindPackageShare('reach_roscon_2023'), '3_mobile_manipulator_reach_study', 'resources', 'reach_study.srdf'])},
  {'name': 'robot_description_kinematics_file',     'description': 'Path to the MoveIt kinematics file',              'default': PathJoinSubstitution([FindPackageShare('reach_roscon_2023'), '3_mobile_manipulator_reach_study', 'resources', 'kinematics.yaml'])},
  {'name': 'robot_description_joint_limits_file',   'description': 'Path to the MoveIt joint limits file',            'default': PathJoinSubstitution([FindPackageShare('reach_roscon_2023'), '3_mobile_manipulator_reach_study', 'resources', 'joint_limits.yaml'])},
  {'name': 'use_rviz',                              'description': 'Flag indicating whether Rviz should be launchd',  'default': 'True'},
  {'name': 'rviz_config',                           'description': 'Reach study Rviz configuration',                  'default': PathJoinSubstitution([FindPackageShare('reach_roscon_2023'), '3_mobile_manipulator_reach_study', 'resources', 'reach_study_config.rviz'])},
]


def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    robot_description_file = LaunchConfiguration('robot_description_file')
    robot_description_semantic_file = LaunchConfiguration('robot_description_semantic_file')
    robot_description_kinematics_file = LaunchConfiguration('robot_description_kinematics_file')
    robot_description_joints_limits_file = LaunchConfiguration('robot_description_joint_limits_file')

    robot_description = ParameterValue(Command(['xacro ', robot_description_file]), value_type=str)
    robot_description_semantic = load_file(robot_description_semantic_file.perform(context))
    kinematics_yaml = load_yaml(robot_description_kinematics_file.perform(context))
    joint_limits_yaml = load_yaml(robot_description_joints_limits_file.perform(context))

    nodes = [
      # Robot state publisher
      Node(package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           output='screen',
           parameters=[{'robot_description': robot_description}]),
      # Joint state publisher
      Node(package='joint_state_publisher',
           executable='joint_state_publisher',
           name='joint_state_publisher',
           output='screen',
           parameters=[{'source_list': ParameterValue(['reach_joints'])}]),
      # Rviz
      Node(condition=IfCondition(LaunchConfiguration('use_rviz')),
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           arguments=['-d', LaunchConfiguration('rviz_config')],
           parameters=[{
              'robot_description': robot_description,
              'robot_description_semantic': robot_description_semantic,
              'robot_description_kinematics': kinematics_yaml,
              'robot_description_planning': joint_limits_yaml,
              }],
           output='screen')
    ]

    return nodes
