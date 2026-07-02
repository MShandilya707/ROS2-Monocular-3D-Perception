import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_dir = get_package_share_directory('depth_to_pointcloud')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz2'
    )

    # Lifecycle node
    perception_node = LifecycleNode(
        package='depth_to_pointcloud',
        executable='perception_node',
        name='depth_to_pointcloud',
        namespace='',
        output='screen',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'sor_k_neighbors': 50,
            'sor_stddev_mult': 1.0
        }]
    )

    # Emit event to configure the node automatically once launched
    emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(perception_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # When it reaches the 'inactive' state (configured), automatically activate it
    register_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=perception_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(perception_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # RViz2 node
    rviz_config_path = PathJoinSubstitution([pkg_dir, 'rviz', 'pointcloud.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # AI Monocular Depth Node
    ai_depth_node = Node(
        package='monocular_depth_estimator',
        executable='webcam_depth_node',
        name='webcam_depth_node',
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        ai_depth_node,
        perception_node,
        emit_configure_event,
        register_activate_handler,
        rviz_node
    ])
