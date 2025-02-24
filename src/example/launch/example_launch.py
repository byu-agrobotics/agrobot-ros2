import launch
import launch_ros.actions
import launch_ros.descriptions

# For more examples, see the CoUGARs repositories:
# https://github.com/BYU-FRoSt-Lab/CoUGARs.git
# https://github.com/BYU-FRoSt-Lab/cougars-ros2.git
# https://github.com/BYU-FRoSt-Lab/cougars-teensy.git
# They may be a bit more complicated than needed for this though.

param_file = '/home/agrobot/config/robot_params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='example',
            executable='talker',
            parameters=[param_file], # An efficient way to pass parameters to the node on startup
            output='screen', # This lets you see print() or cout messages in the terminal
        ),
        launch_ros.actions.Node(
            package='example',
            executable='listener',
            parameters=[param_file],
        ),
    ])