# ROS2 and Gazebo launch file for the Differential Drive Robot
# This launch file will spawn the robot model in Gazebo and set up the necessary nodes for

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    
    # This name has to match the robot name in the Xacro file
    robotXacroName = 'differential_drive_robot'

    # Following is the name of our package, at the same time it is the name of the folder that will be used to define paths
    namePackage = 'mobile_robot'

    # below is the relative path to the Xacro file defining the model
    modelFileRelativePath = 'model/robot.xacro'

    # This is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)


    # get the robot description from the xacro model file

    robotDescription = xacro.process_file(pathModelFile).toxml()


    # This is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))


    # This is the launch description
    # Assuming you are using an empty world model
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown':'true'}.items())


    # Gazebo Node, responsible for spawning the robot into the gazebo simulator
    # note the arguments pass the absolute path of the robot_model, and the topic to which the 
    # robot_description is published from ROS towards gazebo
    # The list of arguments is in fact a sequence of tokens that make up a bash command
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName, '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Robot state publisher node
    # Responsible for publishing the robot state
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # This is very important so we can control the robot from ROS2
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    # Finally we create a launch description object
    LaunchDescriptionObject = LaunchDescription()

    # We add gazebo launch
    LaunchDescriptionObject.add_action(gazeboLaunch)

    # And we add the nodes
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)


    # And we return the LaunchDescription Object
    return LaunchDescriptionObject
