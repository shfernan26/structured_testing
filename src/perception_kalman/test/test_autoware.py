from common.msg import FilteredObjectMsg
from common.msg import AssociatedObjectMsg
from perception_kalman.perception_kalman import KF_Node
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
import time
import unittest

import subprocess
from subprocess import DEVNULL, STDOUT
import nml_bag
import time
import psutil
from os import getpid

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
    start = 0.0 
    cutoff1_start = 0.0 
    cutoff1_end = 0.0 
    cutoff2_start = 0.0 
    cutoff2_end = 0.0 
    total_test_duration = 0.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        print(f"Test duration: {TestKFNode.total_test_duration - (TestKFNode.cutoff1_end-TestKFNode.cutoff1_start) - (TestKFNode.cutoff2_end-TestKFNode.cutoff2_start)}") 

    def setUp(self):
        TestKFNode.start = time.time()
        self.msgs = []
        self.kf_node = KF_Node()
        self.node = rclpy.create_node('test_node')
        self.callback_called = False

        self.obj_pub = self.node.create_publisher(
            msg_type=AssociatedObjectMsg,
            topic='associated_object',
            qos_profile=10
        )

        self.sub = self.node.create_subscription(
            msg_type=FilteredObjectMsg,
            topic='filtered_obj',
            callback=self._msg_received,
            qos_profile=10
        )

        self.addCleanup(self.node.destroy_subscription, self.sub)

    def tearDown(self):
        TestKFNode.cutoff2_start = time.time()
        TestKFNode.get_usage_stats()
        TestKFNode.cutoff2_end = time.time()
        self.kf_node.destroy_node()
        self.node.destroy_node()
        TestKFNode.total_test_duration = time.time() - TestKFNode.start
               

    def _msg_received(self, msg):
        # Callback for ROS 2 subscriber used in the test
        self.msgs.append(msg)
    
    def get_usage_stats():
        my_process = psutil.Process(getpid())
        print("Name:", my_process.name())
        print("PID:", my_process.pid)
        print("Executable:", my_process.exe())
        print("CPU%:", my_process.cpu_percent(interval=1))
        print("MEM%:", my_process.memory_percent())

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

    def test_object_published(self):
        empty_obj = AssociatedObjectMsg()
        empty_obj.obj_count = 0
        self.obj_pub.publish(empty_obj)
        msg = self.get_message()


    def test_min_max_range(self):
        reader = nml_bag.Reader('/home/sachin/automated-testing-framework/src/structured_testing/test/Test1_2022-08-05-13-21-20/Test1_2022-08-05-13-21-20.db3', topics=['/associated_object'])

        TestKFNode.cutoff1_start = time.time()
        for message_record in reader: 

            ass_msg = AssociatedObjectMsg()
            ass_msg.obj_id = message_record["obj_id"]
            ass_msg.obj_dx = message_record["obj_dx"]
            ass_msg.obj_lane = message_record["obj_lane"]
            ass_msg.obj_vx = message_record["obj_vx"]
            ass_msg.obj_dy = message_record["obj_dy"]
            ass_msg.obj_ax = message_record["obj_ax"]
            ass_msg.obj_path = message_record["obj_path"]
            ass_msg.obj_vy = message_record["obj_vy"]
            ass_msg.obj_timestamp = message_record["obj_timestamp"]
            ass_msg.obj_count = message_record["obj_count"]
            ass_msg.obj_source = message_record["obj_source"]
            self.obj_pub.publish(ass_msg)

            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(self.kf_node, timeout_sec=0.1)

        TestKFNode.cutoff1_end = time.time()

        minVal = 30
        maxVal = 85
        dx_list = []
        for msg in self.msgs:
            dx_list.append(msg.obj_dx)

        self.assertGreater(min(dx_list), minVal) 
        self.assertLess(max(dx_list), maxVal) 

