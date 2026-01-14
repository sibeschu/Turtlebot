from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            remappings=[
                ('image', 'image_raw'),
                ('image_rect', 'image_rect_color')
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::CropDecimateNode',
            name='crop_decimate_node',
            remappings=[
                ('in/image_raw', 'image_rect_color'),
                ('out/image_raw', 'image_downsized')
            ],
            parameters={
                'decimation_x': 2,
                'decimation_y': 2,
            }
        )
    ]

    container = ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )

    return LaunchDescription([container])