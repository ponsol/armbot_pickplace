
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("armbot_moveit2", package_name="armbot_moveit2").to_dict()
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()


    pick_place_demo = Node(
        package="armbot_pickplace",
        executable="armbot_pickplace",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
