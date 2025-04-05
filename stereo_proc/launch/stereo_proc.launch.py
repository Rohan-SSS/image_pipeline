from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                'disp_min': LaunchConfiguration('disp_min'),
                'disp_max': LaunchConfiguration('disp_max'),
                'support_threshold': LaunchConfiguration('support_threshold'),
                'support_texture': LaunchConfiguration('support_texture'),
                'candidate_stepsize': LaunchConfiguration('candidate_stepsize'),
                'incon_window_size': LaunchConfiguration('incon_window_size'),
                'incon_threshold': LaunchConfiguration('incon_threshold'),
                'incon_min_support': LaunchConfiguration('incon_min_support'),
                'add_corners': LaunchConfiguration('add_corners'),
                'grid_size': LaunchConfiguration('grid_size'),
                'beta': LaunchConfiguration('beta'),
                'gamma': LaunchConfiguration('gamma'),
                'sigma': LaunchConfiguration('sigma'),
                'sradius': LaunchConfiguration('sradius'),
                'match_texture': LaunchConfiguration('match_texture'),
                'lr_threshold': LaunchConfiguration('lr_threshold'),
                'speckle_sim_threshold': LaunchConfiguration('speckle_sim_threshold'),
                'speckle_size': LaunchConfiguration('speckle_size'),
                'ipol_gap_width': LaunchConfiguration('ipol_gap_width'),
                'filter_median': LaunchConfiguration('filter_median'),
                'filter_adaptive_mean': LaunchConfiguration('filter_adaptive_mean'),
                'postprocess_only_left': LaunchConfiguration('postprocess_only_left'),
                'subsampling': LaunchConfiguration('subsampling'),
            }],
            remappings=[
                ('left/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
                ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('right/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),
                ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
            ]
        )
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            name='approximate_sync', default_value='True',
            description='Whether to use approximate synchronization of topics. Set to true if '
                        'the left and right cameras do not produce exactly synced timestamps.'
        ),
        DeclareLaunchArgument(
            name='use_system_default_qos', default_value='False',
            description='Use the RMW QoS settings for the image and camera info subscriptions.'
        ),
        DeclareLaunchArgument(
            name='launch_image_proc', default_value='True',
            description='Whether to launch debayer and rectify nodes from image_proc.'
        ),
        DeclareLaunchArgument(
            name='left_namespace', default_value='left',
            description='Namespace for the left camera'
        ),
        DeclareLaunchArgument(
            name='right_namespace', default_value='right',
            description='Namespace for the right camera'
        ),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        # Stereo algorithm parameters
        DeclareLaunchArgument(
            name='stereo_algorithm', default_value='0',
            description='Stereo algorithm: ELAS Matching (0)'
        ),
        DeclareLaunchArgument(
            name='disp_min', default_value='0',
            description='Minimum disparity value to consider in the search range'
        ),
        DeclareLaunchArgument(
            name='disp_max', default_value='255',
            description='Maximum disparity value to consider in the search range'
        ),
        DeclareLaunchArgument(
            name='support_threshold', default_value='0.85',
            description='Uniqueness ratio threshold for support point matches'
        ),
        DeclareLaunchArgument(
            name='support_texture', default_value='10',
            description='Minimum texture intensity for a point to be considered a support point'
        ),
        DeclareLaunchArgument(
            name='candidate_stepsize', default_value='5',
            description='Grid step size for selecting support point candidates'
        ),
        DeclareLaunchArgument(
            name='incon_window_size', default_value='5',
            description='Window size for checking inconsistent support points'
        ),
        DeclareLaunchArgument(
            name='incon_threshold', default_value='5',
            description='Disparity similarity threshold for support point consistency'
        ),
        DeclareLaunchArgument(
            name='incon_min_support', default_value='5',
            description='Minimum number of consistent support points required'
        ),
        DeclareLaunchArgument(
            name='add_corners', default_value='0',
            description='Flag to add support points at image corners using nearest-neighbor disparities'
        ),
        DeclareLaunchArgument(
            name='grid_size', default_value='20',
            description='Neighborhood size for extrapolating additional support points'
        ),
        DeclareLaunchArgument(
            name='beta', default_value='0.02',
            description='Weight parameter for image likelihood in cost computation'
        ),
        DeclareLaunchArgument(
            name='gamma', default_value='3',
            description='Prior constant for smoothness regularization'
        ),
        DeclareLaunchArgument(
            name='sigma', default_value='1',
            description='Prior sigma (smoothness) for disparity gradients'
        ),
        DeclareLaunchArgument(
            name='sradius', default_value='2',
            description='Radius for prior sigma smoothing'
        ),
        DeclareLaunchArgument(
            name='match_texture', default_value='1',
            description='Minimum texture for dense matching (avoids low-texture areas)'
        ),
        DeclareLaunchArgument(
            name='lr_threshold', default_value='2',
            description='Threshold for left-right disparity consistency check'
        ),
        DeclareLaunchArgument(
            name='speckle_sim_threshold', default_value='1',
            description='Similarity threshold for speckle region segmentation'
        ),
        DeclareLaunchArgument(
            name='speckle_size', default_value='200',
            description='Maximum size (in pixels) of speckle regions to be removed'
        ),
        DeclareLaunchArgument(
            name='ipol_gap_width', default_value='3',
            description='Maximum gap width (in pixels) to interpolate missing disparities'
        ),
        DeclareLaunchArgument(
            name='filter_median', default_value='0',
            description='Flag to enable median filtering for postprocessing'
        ),
        DeclareLaunchArgument(
            name='filter_adaptive_mean', default_value='1',
            description='Flag to enable adaptive mean filtering for postprocessing'
        ),
        DeclareLaunchArgument(
            name='postprocess_only_left', default_value='1',
            description='Flag to skip right image postprocessing for speed'
        ),
        DeclareLaunchArgument(
            name='subsampling', default_value='0',
            description='Flag to compute disparities at half-resolution for speed'
        ),
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            package='rclcpp_components',
            executable='component_container',
            name='stereo_proc_container',
            namespace='',
            composable_node_descriptions=composable_nodes,
        ),
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', ''),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration('container'),
        ),
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals('container', ''),
            name='container',
            value=PythonExpression([
                '"stereo_proc_container"', ' if ',
                '"', LaunchConfiguration('ros_namespace', default=''), '"',
                ' == "" else ', '"',
                LaunchConfiguration('ros_namespace', default=''), '/stereo_proc_container"'
            ]),
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('left_namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'container': LaunchConfiguration('container')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('right_namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'container': LaunchConfiguration('container')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        )
    ])