import reach
import reach_ros
from pnp_target_pose_generator import PnPTargetPoseGenerator

from ament_index_python import get_package_share_path
from launch.substitutions import Command
from launch import LaunchContext
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import os
import rclpy
import rclpy.node
from tempfile import NamedTemporaryFile
import yaml


REACH_PLUGIN_LIBS = ('reach_plugins', 'reach_ros_plugins', 'reach_roscon_2023_plugins')


def run_reach_study(config_file: str):
    # Load the file
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    params = reach.Parameters()
    opt_config = config["optimization"]
    params.radius = opt_config["radius"]
    params.max_steps = opt_config["max_steps"]
    params.step_improvement_threshold = opt_config["step_improvement_threshold"]

    os.environ[reach.SEARCH_LIBRARIES_ENV] = os.pathsep.join(REACH_PLUGIN_LIBS)
    loader = reach.PluginLoader()
    loader.search_libraries_env = reach.SEARCH_LIBRARIES_ENV

    # Load the IK solver from a c++ plugin
    ik_config = config['ik_solver']
    ik_solver_factory = loader.createIKSolverFactoryInstance(ik_config["name"])
    ik_solver = ik_solver_factory.create(ik_config)

    # Load the evaluator from a c++ plugin
    eval_config = config['evaluator']
    evaluator_factory = loader.createEvaluatorFactoryInstance(eval_config["name"])
    evaluator = evaluator_factory.create(eval_config)

    # Load the logger from a c++ plugin
    display_config = config['display']
    display_factory = loader.createDisplayFactoryInstance(display_config["name"])
    display = display_factory.create(display_config)

    # Load the logger from a c++ plugin
    logger_config = config['logger']
    logger_factory = loader.createLoggerFactoryInstance(logger_config["name"])
    logger = logger_factory.create(logger_config)

    # Create the target pose generator
    target_pose_generator = PnPTargetPoseGenerator(
        node=rclpy.node.Node('tpg_node'),
        **config['target_pose_generator'],
    )

    study = reach.ReachStudy(ik_solver, evaluator, target_pose_generator, display, logger, params)
    study.run()
    study.optimize()

    # Save the results to file
    db_file = '/tmp/reach_study.db.xml'
    print(f'Saving reach study results to \'{db_file}\'')
    study.save(db_file)

    # Show the results
    db = study.getDatabase()
    results = db.calculateResults()
    logger.printResults(results)
    display.showEnvironment()
    display.showResults(db.results[-1])

    input('Reach study complete. Press enter to continue...')


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


def init_ros(params: dict):
    # Initialize the static node in `reach_ros` with the required parameters
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=True) as h:
        lc = LaunchContext()
        norm_params = normalize_parameters([params])
        eval_params = evaluate_parameters(lc, norm_params)
        param_dict = {
            '/**':
                {'ros__parameters': eval_params[0]}
        }
        yaml.dump(param_dict, h, default_flow_style=False)
        reach_ros.init_ros(['--ros-args', '--params-file', h.name])

    # Initialize rclpy in order to make a node and use TF
    rclpy.init()


def main():
    reach_roscon_2023_path = os.path.join(get_package_share_path('reach_roscon_2023'), '2_pick_and_place_reach_study')

    # Define the reach study configuration file
    config_file = os.path.join(reach_roscon_2023_path, 'reach_study.yaml')

    # Define the ROS parameters needed for the c++ plugins
    xacro_file = os.path.join(reach_roscon_2023_path, 'resources', 'reach_study.xacro')
    srdf_file = os.path.join(reach_roscon_2023_path, 'resources', 'reach_study.srdf')
    kinematics_file = os.path.join(reach_roscon_2023_path, 'resources', 'kinematics.yaml')
    joint_limits_file = os.path.join(reach_roscon_2023_path, 'resources', 'joint_limits.yaml')

    urdf_param = Command(['xacro ', xacro_file])
    srdf_param = load_file(srdf_file)
    kin_param = load_yaml(kinematics_file)
    joint_limits_param = load_yaml(joint_limits_file)

    # Define parameters needed for the reach_ros plugins
    parameters = {
        'robot_description': ParameterValue(urdf_param, value_type=str),
        'robot_description_semantic': srdf_param,
        'robot_description_kinematics': kin_param,
        'robot_description_planning': joint_limits_param,
    }
    init_ros(parameters)

    # Run the reach study
    run_reach_study(config_file)


if __name__ == '__main__':
    main()
