from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # known vessel ids = 
    known_vessel_ids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR','RAS_TN_YE','RAS_TN_RE','RAS_TN_LB','RAS_TN_PU']
    # Get vessel id from os environment $VESSEL_ID
    vesselid = os.environ['VESSEL_ID']

    # Check if vessel id is known, else throw an error
    if vesselid not in known_vessel_ids:
        raise ValueError("Unknown vessel id: ", vesselid,'" Please check the vessel id in the environment variable $VESSEL_ID.')

    # Print: start formation control launch generation:
    print("Start formation control launch generation")
    boat_controller_dir = get_package_share_directory('boat_controller')
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(boat_controller_dir + '/single_boid.launch.py'))
                
    ld.add_action(included_launch)

    turtleboat_node = Node(
        package='turtleboat',
        executable='turtleboatmain',
        name='turtleboat_sim',
        namespace=vesselid,
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(turtleboat_node)

    return ld
