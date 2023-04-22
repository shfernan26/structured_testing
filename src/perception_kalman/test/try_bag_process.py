import subprocess
from subprocess import DEVNULL, STDOUT
import signal
import sys
import rclpy
from rclpy.node import Node
import time
from common.msg import AssociatedObjectMsg

class MY_Node(Node):
    def __init__(self):
        super().__init__("perception_kalman")
        self.objects = {}
        print("whatup")
        self.callback_called =  False
        self.sub = self.create_subscription(AssociatedObjectMsg, 'associated_object',
                                 callback=self.obj_association_callback, qos_profile=10)
        
    def obj_association_callback(self, obj):
        self.callback_called =  True





bag_processes = []
rclpy.init()
kf_node = MY_Node()

start_command = "ros2 bag play src/structured_testing/test/Test1_2022-08-05-13-21-20/Test1_2022-08-05-13-21-20.db3 --topics /associated_object"

bag_processes.append(
        subprocess.Popen(
            start_command, shell=True, stdout=DEVNULL, stderr=STDOUT
        )
    )

# Give the ros2bag time to start
time.sleep(5)

rclpy.spin_once(kf_node, timeout_sec=0.1)

if not kf_node.callback_called:
    raise RuntimeError("Failed to start rosbag")
else:
    print("Playing rosbag: " + start_command)
    for process in bag_processes:
        process.terminate()
        process.wait()

kf_node.destroy_subscription(kf_node.sub)
kf_node.destroy_node()