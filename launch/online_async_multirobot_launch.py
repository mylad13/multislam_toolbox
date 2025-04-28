import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    resolution = LaunchConfiguration('resolution')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_robot_name_argument = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot Name / Namespace')
    
    declare_resolution_argument = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the map')
    
    # Common remappings for all nodes
    common_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/map_metadata', 'map_metadata')
    ]

    # For high-resolution slam: publish on /map (i.e. no extra remapping needed)
    high_res_remappings = common_remappings + [('/map', 'map')]
    # For low-resolution slam: publish on /map_low instead
    low_res_remappings = common_remappings + [
        ('/map', 'map'), 
        # ('/localized_scan', '/localized_scan_lowres')
    ]

    # High resolution node (only runs if resolution != '1')
    high_res_slam = Node(
        package='slam_toolbox',
        executable='multirobot_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        remappings=high_res_remappings,
        parameters=[
            ParameterFile(os.path.join(pkg_dir, 'config', 'mapper_params_online_multi_async.yaml'), allow_substs=True),
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(PythonExpression(["'", resolution, "' == '1'"]))
    )

    # Low resolution node (only runs if resolution == '1')
    low_res_slam = Node(
        package='slam_toolbox',
        executable='multirobot_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        remappings=low_res_remappings,
        parameters=[
            ParameterFile(os.path.join(pkg_dir, 'config', 'mapper_params_online_multi_async_lowres.yaml'), allow_substs=True),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(PythonExpression(["'", resolution, "' == '1'"]))
    )
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_resolution_argument)
    ld.add_action(high_res_slam)
    ld.add_action(low_res_slam)

    return ld
