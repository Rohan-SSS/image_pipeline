import os
import sys
import time
import unittest
import numpy as np
import cv2
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from stereo_msgs.msg import DisparityImage

@pytest.mark.rostest
def generate_test_description():
    path_to_stereo_image_publisher_fixture = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'stereo_image_publisher.py')
    path_to_test_data = os.path.join(os.path.dirname(__file__), 'data')
    path_to_left_image = os.path.join(path_to_test_data, 'aloe-L.png')
    path_to_right_image = os.path.join(path_to_test_data, 'aloe-R.png')

    return LaunchDescription([
        Node(
            executable=sys.executable,
            arguments=[
                path_to_stereo_image_publisher_fixture,
                path_to_left_image,
                path_to_right_image
            ],
            output='screen'
        ),
        Node(
            package='stereo_proc',
            executable='disp_node',
            name='disp_node',
            # ELAS Defaults (MIDDLEBURRY)
            parameters=[
                {'use_system_default_qos': True},
                {'disp_min': 0},
                {'disp_max': 255},
                {'support_threshold': 0.95},
                {'support_texture': 10},
                {'candidate_stepsize': 5},
                {'incon_window_size': 5},
                {'incon_threshold': 5},
                {'incon_min_support': 5},
                {'add_corners': 1},
                {'grid_size': 20},
                {'beta': 0.02},
                {'gamma': 5.0},
                {'sigma': 1.0},
                {'sradius': 3.0},
                {'match_texture': 0},
                {'lr_threshold': 2},
                {'speckle_sim_threshold': 1.0},
                {'speckle_size': 200},
                {'ipol_gap_width': 5000},
                {'filter_median': 1},
                {'filter_adaptive_mean': 0},
                {'postprocess_only_left': 0},
                {'subsampling': 0}
            ],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestDisparityNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_disparity_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_message_received(self):
        msgs_received = []
        self.node.create_subscription(
            DisparityImage,
            'disparity',
            lambda msg: msgs_received.append(msg),
            1
        )

        # Wait for 60s
        start_time = time.time()
        while len(msgs_received) == 0 and (time.time() - start_time) < 60:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        assert len(msgs_received) > 0, "No disparity message received!"

        # Load expected disparity
        path_to_test_data = os.path.join(os.path.dirname(__file__), 'data')
        expected_disp_path = os.path.join(path_to_test_data, 'aloe-disp.png')
        expected_disp = cv2.imread(expected_disp_path, cv2.IMREAD_GRAYSCALE)

        # Convert received disparity to numpy array
        received_disp = self._disparity_msg_to_numpy(msgs_received[0])

        # Save received disparity for debugging
        received_disp_path = os.path.join(path_to_test_data, 'received_disp.png')
        self._save_disparity_image(received_disp, received_disp_path)
        print(f"Saved received disparity to: {received_disp_path}")

        if expected_disp.shape != received_disp.shape:
            expected_disp = cv2.resize(expected_disp, (received_disp.shape[1], received_disp.shape[0]))

        mse = np.mean((expected_disp - received_disp) ** 2)
        self.assertLess(mse, 10.0, f"Disparity mismatch! MSE = {mse}")

    def _disparity_msg_to_numpy(self, msg: DisparityImage) -> np.ndarray:
        disp_data = np.frombuffer(msg.image.data, dtype=np.float32)
        disp_data = disp_data.reshape(msg.image.height, msg.image.width)
        return disp_data

    def _save_disparity_image(self, disparity: np.ndarray, file_path: str):
        disp_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imwrite(file_path, disp_normalized)