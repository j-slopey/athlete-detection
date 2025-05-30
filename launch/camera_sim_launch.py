import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_field_camera_sim = get_package_share_directory('field_camera_sim')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')

    model_path = os.path.join(pkg_field_camera_sim, 'models')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_field_camera_sim, 'worlds', 'field.sdf')}.items(),
    )

    num_dummies = DeclareLaunchArgument(
        'num_dummies',
        default_value='3',
        description='Number of dummy models to spawn in the simulation.'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    simulation_movements = Node(
        package='field_camera_sim',
        executable='simulation_movements',
        parameters=[{
                'num_dummies': LaunchConfiguration('num_dummies')
            }]
    )
    

    object_detection = Node(
        package='field_camera_sim',
        executable='object_detection',
        output='screen'
    )

    return LaunchDescription([
        num_dummies,

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                os.pathsep + model_path
            ]
        ),
       
        gazebo_launch,
        gz_bridge,
        simulation_movements,
        object_detection
    ])