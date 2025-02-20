from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # 定义节点
    armor_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector'
    )

    pnp_solver = Node(
        package='pnp_solver',
        executable='pnp_solver_node',
        name='pnp_solver'
    )

    ballistic_calculator = Node(
        package='ballistic_calculator',
        executable='ballistic_calculator_node',
        name='ballistic_calculator'
    )

    # 设置启动顺序
    return LaunchDescription([
        armor_detector,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=armor_detector,
                on_start=[pnp_solver]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=pnp_solver,
                on_start=[ballistic_calculator]
            )
        )
    ])