from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()

    turtle1_ns = "Peemai"
    turtle2_ns = "killer"

    sampling_frequency_arg = DeclareLaunchArgument(
        'sampling_frequency',
        default_value='100.0',
        description='Sampling frequency for eater and killer nodes in Hz'
    )

    eater_name_arg = DeclareLaunchArgument(
        'eater_name',
        default_value='eater_turtle',
        description='Target name for killer node'
    )

    # Get launch configurations
    sampling_frequency = LaunchConfiguration('sampling_frequency')
    eater_name = LaunchConfiguration('eater_name')

    Turtlesim =  Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
        )
    
    Eater = Node(
            package='lab3',
            namespace= turtle1_ns ,
            executable='eater.py',
            name='Eater',
            parameters = [
                {'sampling_frequency' : sampling_frequency}
            ]
        )
    
    Killer = Node(
            package='lab3',
            namespace = turtle2_ns,
            executable='killer.py',
            name='Killer',
            parameters = [
                {'sampling_frequency' : sampling_frequency},
                {'target_name' : turtle1_ns}
            ],
        )
    
    Kill_turtle1 = ExecuteProcess(
        cmd = [['ros2 service call /remove_turtle turtlesim/srv/Kill ' + "'name: 'turtle1''"]],
        shell=True
    )
    Spawn_turtle1 = ExecuteProcess(
        cmd = [['ros2 service call /spawn_turtle turtlesim/srv/Spawn ' + f"'name: '{turtle1_ns}''"]],
        shell=True
    )
    Spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn_turtle turtlesim/srv/Spawn ' + f"'name: '{turtle2_ns}''"]],
        shell=True
    )

    launch_description.add_action(sampling_frequency_arg)
    launch_description.add_action(eater_name_arg)
    launch_description.add_action(Turtlesim)
    # launch_description.add_action(Eater)
    # launch_description.add_action(Killer)
    launch_description.add_action(Kill_turtle1)
    launch_description.add_action(Spawn_turtle1)
    launch_description.add_action(Spawn_turtle2)
    launch_description.add_action(Eater)
    launch_description.add_action(Killer)

    return launch_description
