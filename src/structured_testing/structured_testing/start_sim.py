#!/usr/bin/env python

import rclpy
import sys
import signal
import subprocess
import argparse
import time
import psutil
from os import getpid

from subprocess import DEVNULL, STDOUT

# import rosbag
from ros2bag_helper import getROS2BagData  # Using custom ros2bag reader script

from observers import ObserverManager

from sim_yaml_parser import getYAMLData, dumpOrderedDict

START_TIME = 0.0
END_TIME = 0.0
CUTOFF1_START = 0.0
CUTOFF1_END = 0.0
CUTOFF2_START = 0.0
CUTOFF2_END = 0.0


def get_usage_stats():
    my_process = psutil.Process(getpid())
    print("Name:", my_process.name())
    print("PID:", my_process.pid)
    print("Executable:", my_process.exe())
    print("CPU%:", my_process.cpu_percent(interval=1))
    print("MEM%:", my_process.memory_percent())


class LaunchProcessManager:

    

    def __init__(self, launch_files, bags, topics, observers, results_file):
        global CUTOFF1_START
        global CUTOFF1_END
        global CUTOFF2_START
        self.launch_files = launch_files
        self.topics = topics
        self.bags = bags
        self.bag_to_topic = self.generateGetBagTopicMap()
        self.bag_processes = []
        self.launch_processes = []
        CUTOFF1_START = time.time()
        self.startBagPlayback()
        CUTOFF1_END = time.time()
        self.startLaunchProcess()
        self.observer_manager = ObserverManager()
        for obs in observers:
            self.observer_manager.addObserver(obs)
        self.results_file = results_file
        CUTOFF2_START = time.time()

    def startBagPlayback(self):
        for bag, topics in self.bag_to_topic.items():
            topic_string = str(topics)  # Of the form {'A', 'B'}
            topic_string = topic_string.lstrip("{").rstrip(
                "}"
            )  # Remove braces looks like 'A', 'B'
            topic_string = topic_string.replace(
                ",", " "
            )  # remove comma looks like 'A' 'B'
            start_command = f"ros2 bag play {bag.path} --topics {topic_string}"
            print(start_command)
            self.bag_processes.append(
                subprocess.Popen(
                    start_command, shell=True, stdout=DEVNULL, stderr=STDOUT
                )
            )

    def killBagPlayback(self):
        for p in self.bag_processes:
            p.kill()

    def killLaunchFiles(self):
        for p in self.launch_processes:
            p.kill()

    def killObservers(self):
        observer_results = self.observer_manager.getResults()
        dumpOrderedDict(observer_results, self.results_file)
        self.observer_manager.destroy_node()

    def kill(self):
        global CUTOFF2_END
        get_usage_stats()
        CUTOFF2_END = time.time()
        self.killBagPlayback()
        self.killLaunchFiles()
        self.killObservers()

    def startLaunchProcess(self):
        for lf in self.launch_files:
            start_command = f"ros2 launch {lf.path}"
            print(start_command)
            self.launch_processes.append(
                subprocess.Popen(
                    start_command, shell=True, stdout=DEVNULL, stderr=STDOUT
                )
            )

    def generateGetBagTopicMap(self):
        self.bag_to_topic = dict()
        for topic in self.topics:
            bag = self.getBagWithTopic(topic)
            if bag in self.bag_to_topic:
                self.bag_to_topic[bag].add(topic)
            else:
                self.bag_to_topic[bag] = set([topic])
        return self.bag_to_topic

    def getBagWithTopic(self, topic):
        for bag in self.bags:
            if topic in bag.bagged_topics:
                return bag
        raise Exception(f"Topic {topic} not found in any bag")


class Bagfile:
    def __init__(self, file_path):
        self.path = file_path
        self.bagged_topics = getROS2BagData(file_path)


class Launchfile:
    def __init__(self, file_path):
        self.path = file_path

    def __str__(self):
        return self.path


class LaunchManager:
    def __init__(self):
        self.launch_files = set()
        self.published_topics = set()
        self.required_topics = set()
        self.bagged_topics = set()
        self.bags = []

    def addLaunchFile(self, file_path):
        if file_path in self.launch_files:
            raise Exception(
                f"Launchfile {file_path} already registered to launch manager"
            )
        self.launch_files.add(Launchfile(file_path))

    def addBagFile(self, file_path):
        if file_path in self.bags:
            raise Exception(f"Bagfile {file_path} already registered to launch manager")
        self.bags.append(Bagfile(file_path))

    def addLaunchFiles(self, lf_list):
        for lf in lf_list:
            self.addLaunchFile(lf)

    def addBagFiles(self, bag_list):
        for b in bag_list:
            self.addBagFile(b)

    def addTopicsFromSimFile(self, pub_topics, req_topics):
        self.published_topics = pub_topics
        self.required_topics = req_topics

    def extractTopicsFromBags(self):
        res = set()
        for bag in self.bags:
            res |= bag.bagged_topics
        self.bagged_topics = res

    def getRequiredTopicsFromBag(self):
        missing_topics = self.required_topics - self.published_topics
        if missing_topics <= self.bagged_topics:
            return missing_topics
        raise Exception(
            f"Launch Files require {missing_topics} but all topics not found in bags. "
        )


lpm = None


def signal_handler(sig, frame):
    global END_TIME
    print("Killing all Structured Testing ", time.time())
    lpm.kill()
    rclpy.try_shutdown()
    total_test_duration = time.time() - START_TIME
    print(f"Test duration: {total_test_duration - (CUTOFF1_END-CUTOFF1_START) - (CUTOFF2_END-CUTOFF2_START)}")    
    sys.exit(0)
    


def main(simFile):
    global lpm
    global START_TIME
    rclpy.init(args=sys.argv)
    START_TIME = time.time()
    rclpy.create_node('SimulationManager')
    lm = LaunchManager()
    launch_files, bag_files, duration, observers, pub_topics, req_topics = getYAMLData(simFile)
    results_file = simFile + "results"  # example.sim => example.simresults
    lm.addLaunchFiles(launch_files)
    lm.addBagFiles(bag_files)
    lm.addTopicsFromSimFile(pub_topics, req_topics)
    lm.extractTopicsFromBags()
    signal.alarm(duration)
    signal.signal(signal.SIGALRM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    lpm = LaunchProcessManager(
        lm.launch_files, lm.bags, lm.getRequiredTopicsFromBag(), observers, results_file
    )

    rclpy.spin(lpm.observer_manager)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "simFile", help="The .sim file defining what to launch in the simulation"
    )
    args = parser.parse_args()
    main(args.simFile)
