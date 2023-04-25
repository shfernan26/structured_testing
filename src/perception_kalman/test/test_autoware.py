from common.msg import FilteredObjectMsg
from common.msg import AssociatedObjectMsg
from perception_kalman.perception_kalman import KF_Node
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
import time
import unittest
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import subprocess
from subprocess import DEVNULL, STDOUT


pytest.mark.launch_test
def generate_test_description():
    kf_node = Node(
        package='perception_kalman',
        namespace='perception_kalman1',
        executable='perception_kalman'
    )

    context = {
        'perception_kalman': kf_node,
    }  
    return (
        LaunchDescription([
            kf_node,
            ReadyToTest() # to start test right away
        ]),
        context
    )

class TestKFNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.msgs = []
        self.bag_processes = []
        self.kf_node = KF_Node()
        self.node = rclpy.create_node('test_node')
        self.callback_called = False

        # self.obj_pub = self.node.create_publisher(
        #     msg_type=AssociatedObjectMsg,
        #     topic='associated_object',
        #     qos_profile=10
        # )

        # self.sub = self.node.create_subscription(
        #     msg_type=FilteredObjectMsg,
        #     topic='filtered_obj',
        #     callback=self._msg_received,
        #     qos_profile=10
        # )
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=10
        )

        self.sub = self.node.create_subscription(
            msg_type=AssociatedObjectMsg,
            topic='associated_object',
            callback=self.rosbag_started,
            qos_profile=qos_profile
        )

        self.addCleanup(self.node.destroy_subscription, self.sub)

    def tearDown(self):
        self.kf_node.destroy_node()
        self.node.destroy_node()
        for p in self.bag_processes:
            p.kill()

    def rosbag_started(self):
        self.callback_called =  True

    def _msg_received(self, msg):
        # Callback for ROS 2 subscriber used in the test
        self.msgs.append(msg)

    def get_message(self):
        startlen = len(self.msgs)

        # Try up to 5 s to receive messages
        end_time = time.time() + 5.0
        while time.time() < end_time:
            rclpy.spin_once(self.kf_node, timeout_sec=0.1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if startlen != len(self.msgs):
                break
        self.assertNotEqual(startlen, len(self.msgs))
        return self.msgs[-1]

    def test_topic_name(self):
        topics = self.node.get_topic_names_and_types()
        topic = "/filtered_obj"
        self.assertIn(topic, str(topics))

    # def test_object_published(self):
    #     empty_obj = AssociatedObjectMsg()
    #     empty_obj.obj_count = 0
    #     self.obj_pub.publish(empty_obj)
    #     msg = self.get_message()


    def test_min_max_range(self):
        
        start_command = "ros2 bag play src/structured_testing/test/Test1_2022-08-05-13-21-20/Test1_2022-08-05-13-21-20.db3 --topics /associated_object"

        self.bag_processes.append(
                subprocess.Popen(
                    start_command, shell=True, stdout=DEVNULL, stderr=STDOUT
                )
            )

        # Give the ros2bag time to start
        time.sleep(5)

        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(self.kf_node, timeout_sec=0.1)

        if not self.callback_called:
            raise RuntimeError("Failed to start rosbag")
        else:
            print("Playing rosbag: " + start_command)

        minVal = 30
        maxVal = 90
        dx_list = []
        for msg in self.msgs:
            dx_list.append(msg.obj_dx)

        self.assertGreater(min(dx_list), minVal) 
        self.assertLess(max(dx_list), maxVal) 

