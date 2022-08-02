import rospy
import sys
import os
import signal
import subprocess

from subprocess import DEVNULL, STDOUT, check_call

import rosbag
import rosmaster 
import xmlrpc

from xml.dom.minidom import parse, parseString
from time import sleep
from base_observer import *


from sim_yaml_parser import getYAMLData, dumpOrderedDict


class LaunchProcessManager:

    def __init__(self, launch_files, bags, topics, observers, results_file):
        self.launch_files = launch_files
        self.topics = topics
        self.bags = bags
        self.bag_to_topic = self.generateGetBagTopicMap()
        self.bag_processes = []
        self.launch_processes = []
        self.startBagPlayback()
        self.startLaunchProcess()
        self.observer_manager = ObserverManager()
        for obs in observers:
             self.observer_manager.addObserver(obs)
        self.results_file = results_file
    
    def startBagPlayback(self):
        for bag, topics in self.bag_to_topic.items():
            topic_string = str(topics) # Of the form {'A', 'B'}
            topic_string = topic_string.lstrip("{"). rstrip("}") #Remove braces looks like 'A', 'B'
            topic_string = topic_string.replace(",", " ") #remove comma looks like 'A' 'B'
            start_command = f"rosbag play {bag.path} --topics {topic_string}"
            print(start_command)
            self.bag_processes.append(subprocess.Popen(start_command, shell=True, stdout=DEVNULL, stderr=STDOUT))

    def killBagPlayback(self):
        for p in self.bag_processes:
            p.kill()

    def killLaunchFiles(self):
        for p in self.launch_processes:
            p.kill()

    def killObservers(self):
        observer_results = self.observer_manager.getResults()
        dumpOrderedDict(observer_results, self.results_file)


    def kill(self):
        self.killBagPlayback()
        self.killLaunchFiles()
        self.killObservers()


    def startLaunchProcess(self):
        for lf in self.launch_files:
            start_command = f"roslaunch {lf.path}"
            print(start_command)
            self.launch_processes.append(subprocess.Popen(start_command, shell=True, stdout=DEVNULL, stderr=STDOUT))


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
        self.bagged_topics = self.getBagTopics(file_path)

    def getBagTopics(self, file_path):
        bag = rosbag.Bag(file_path)
        return set(bag.get_type_and_topic_info().topics.keys())

class Launchfile:
    def __init__(self, file_path):
        self.path = file_path
        self.published_topics = self.getTopicsFromXML(file_path, "PublishedTopics")
        self.required_topics = self.getTopicsFromXML(file_path, "RequiredTopics")

    def getTopicsFromXML(self, file_path, typeOfTopic):
        topics = set()
        root = parse(file_path).getElementsByTagName('launch')[0]
    
        topic_list = root.getElementsByTagName(typeOfTopic)
        if not topic_list:
            return topics
        topic_list = topic_list[0].getElementsByTagName("topic")
        for topic in topic_list:
            data = topic.firstChild.nodeValue
            topics.add(data)
        return topics

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
            raise Exception(f"Launchfile {file_path} already registered to launch manager")
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
    
    def extractTopicsFromLaunchFiles(self):
        published_topics = set()
        required_topics = set()
        for lf in self.launch_files:
            published_topics |= lf.published_topics
            required_topics  |= lf.required_topics
        self.published_topics = published_topics
        self.required_topics = required_topics

    def extractTopicsFromBags(self):
        res = set()
        for bag in self.bags:
            res |= bag.bagged_topics
        self.bagged_topics = res

    def getRequiredTopicsFromBag(self):
        missing_topics = self.required_topics - self.published_topics
        if missing_topics <= self.bagged_topics:
            return missing_topics
        raise Exception(f"Launch Files require {missing_topics} but all topics not found in bags. ")
            


lpm = None

def signal_handler(sig, frame):
    print('Killing all Structured Testing')
    lpm.kill()
    sys.exit(0)

def rosUP():
    try:
        m = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        code, msg, val = m.getSystemState("")
        if code != 1:
            return False
        return True
    except Exception as e:
        print(e)
        return False 
    return True

def main():
    global lpm
    if not rosUP():
        raise Exception("Structured Testing started without rosmaster while ensure roscore is running")
    rospy.init_node('SimulationManager')
    lm = LaunchManager()
    launch_files, bag_files, observers = getYAMLData(sys.argv[1])
    results_file = sys.argv[1] + "results" #example.sim => example.simresults
    lm.addLaunchFiles(launch_files)
    lm.addBagFiles(bag_files)
    lm.extractTopicsFromLaunchFiles()
    lm.extractTopicsFromBags()
    print(lm.bagged_topics)
    print(lm.required_topics)
    print(lm.bags)
    signal.signal(signal.SIGINT, signal_handler)

    lpm = LaunchProcessManager(lm.launch_files, lm.bags, lm.getRequiredTopicsFromBag(), observers, results_file)

    rospy.spin()

    # parseLaunchFile(launch_file)


if __name__ == '__main__':
    main()
 