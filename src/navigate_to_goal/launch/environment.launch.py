import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "navigate_to_goal"
    world_file_name = "obstructed_waypoints.world"
    
    # Dynamically construct the path
    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        world_file_name
    )
    
    # Log the world file path for debugging
    log_world = LogInfo(msg=f"Using world file: {world_path}")
    
    # Set environment variable so Gazebo uses the correct world file
    set_world_env = SetEnvironmentVariable(name='GAZEBO_WORLD_FILE', value=world_path)
    
    # Force software rendering to avoid potential GPU issues
    set_libgl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    # Append turtlebot3 gazebo models to GAZEBO_MODEL_PATH
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    turtlebot3_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    new_model_path = f"{current_model_path}:{turtlebot3_model_path}" if current_model_path else turtlebot3_model_path
    set_model_env = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=new_model_path)
    
    # Include the Gazebo launch file with gui enabled
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gui': 'true'}.items(),
    )
    
    return LaunchDescription([
        log_world,
        set_world_env,
        set_libgl,
        gazebo_launch,
        set_model_env,
    ])
