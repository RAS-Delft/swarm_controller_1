from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    known_vessel_ids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR','RAS_TN_YE','RAS_TN_RE','RAS_TN_LB','RAS_TN_PU']
    # Get vessel id from os environment $VESSEL_ID
    vesselid = os.environ['VESSEL_ID']
    # Check if vessel id is known, else throw an error
    if vesselid not in known_vessel_ids:
        raise ValueError("Unknown vessel id: ", vesselid,'" Please check the vessel id in the environment variable $VESSEL_ID.')

    # Print: start formation control launch generation:
    core_control_dir = get_package_share_directory('ras_ros_core_control_modules')
    utils_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(core_control_dir + '/fleet_geo_utils.launch.py'))
    ld.add_action(utils_launch)

    # Start Boid controller for each vessel
    boid_controller_node = Node(
        package=['boat_controller'],
        executable='boat_controllermain',
        name='boid_controller',
        namespace=vesselid,
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(boid_controller_node)

    # Start vessel heading controller for each vessel
    heading_controller_node = Node(
        package=['ras_ros_core_control_modules'],
        executable='heading_controller',
        name='heading_controller',
        namespace=vesselid,
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(heading_controller_node)
    
    # Start vessel surge velocity controller for each vessel
    surge_controller_node = Node(
        package=['ras_ros_core_control_modules'],
        executable='surge_vel_controller',
        name='surge_velocity_controller',
        namespace=vesselid,
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(surge_controller_node)
    
    # Start thrust allocator for each vessel
    thrust_allocator_node = Node(
        package=['ras_ros_core_control_modules'],
        executable='TN_control_effort_allocator_nomoto',
        name='thrust_allocator',
        namespace=vesselid,
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(thrust_allocator_node)

    # Start velocity differentiator for each vessel
    velocity_differentiator_node = Node(
        package=['ras_ros_core_control_modules'],
        executable='vel_differentiator',
        name='velocity_differentiator',
        namespace=vesselid,
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(velocity_differentiator_node)

    ras_urdf_common_path = FindPackageShare('ras_urdf_common')
    default_model_path = PathJoinSubstitution(['urdf', 'titoneri.urdf.xacro'])    

    # Start robot description content publisher
    ## Make urdf description
    robot_description_content = ParameterValue(Command(['xacro ',
                                                    PathJoinSubstitution([ras_urdf_common_path, default_model_path]),
                                                    ' vesselcolor:="',ras_display_tools.vesselcolors_rgb.get_normalized_string(vesselid) + ' 0.9"',]),
                                                    value_type=str)
    
    ## Make another robot description publisher that gets jointstate from /reference/actuation
    robot_state_publisher_node2 = Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                namespace=vesselid,
                                parameters=[{
                                        'robot_description': robot_description_content,
                                        'publish_frequency': 30.0,
                                        'frame_prefix': vesselid + '/',
                                }],
                                remappings=[
                                    ('joint_states', 'reference/actuation'),
                                    ]
                                )
    ld.add_action(robot_state_publisher_node2)

    baselink_arg = vesselid+'/base_link'

    navsat_transform_node = Node(
        package='ras_ros_core_control_modules',
        executable='navsat_cartesian_transform',
        name='geo_cartesian_tf',
        namespace=vesselid,
        parameters=[{
            'datum':[52.00158915434494,4.371940750746264,0],
            'map_frame': 'world',
            'base_link_frame': baselink_arg
        }],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(navsat_transform_node)

    return ld
