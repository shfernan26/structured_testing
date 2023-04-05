from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
 
import os
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():

    association_node = Node(
        package='sensor_fusion',
        executable='data_association_node'
    )

    context = {
        'association_node': association_node,
    }  
    return LaunchDescription([
        association_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
 
    def test_exit_code(self, proc_output, proc_info, ndt_mapper):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=ndt_mapper)


'''
class TestPlanningNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.msgs = []
        self.planning_node = PlanningNode()
        self.node = rclpy.create_node('test_node')
        
        self.obj_pub = self.node.create_publisher(
            msg_type=TrackedObjectArray,
            topic='tracked_obj_array',
            qos_profile=10
        )
        self.sub = self.node.create_subscription(
            msg_type=PlannedTrajectory,
            topic='planned_trajectory',
            callback=self._msg_received,
            qos_profile=10
        )

        self.addCleanup(self.node.destroy_subscription, self.sub)

    def tearDown(self):
        self.planning_node.destroy_node()
        self.node.destroy_node()

    def _msg_received(self, msg):
        # Callback for ROS 2 subscriber used in the test
        self.msgs.append(msg)

    def get_message(self):
        startlen = len(self.msgs)

        # Try up to 5 s to receive messages
        end_time = time.time() + 5.0
        while time.time() < end_time:
            rclpy.spin_once(self.planning_node, timeout_sec=0.1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if startlen != len(self.msgs):
                break

        self.assertNotEqual(startlen, len(self.msgs))
        return self.msgs[-1]

    def test_topic_name(self):
        topics = self.node.get_topic_names_and_types()
        topic = "/planned_trajectory"
        self.assertIn(topic, str(topics))

    def test_trajectory_published(self):
        empty_obj_array = TrackedObjectArray()
        empty_obj_array.objects = []
        empty_obj_array.num_of_objects = 0
        self.obj_pub.publish(empty_obj_array)

        msg = self.get_message()
'''