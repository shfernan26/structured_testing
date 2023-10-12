import sys
import inspect
import yaml
import os
from yaml import SafeDumper
from pathlib import Path
from collections import OrderedDict

# Inline comment on next line tells flake8 test to ignore 'imported but unused' error
from observers import *  # noqa: F401, F403
from ament_index_python.packages import get_package_share_directory


def getLaunchfiles(data):
    lf_data = data["launch_files"]
    file_paths = []
    for lf in lf_data:
        pkg = lf["package"]
        f = lf["file"]
        if not f.endswith(".launch"):
            raise Exception(f"{f} is not a launch file unable to use")
        package_share_directory = get_package_share_directory('structured_testing')
        temp_strg = package_share_directory.replace('install', 'src')
        pkg_path = temp_strg.replace('/share/structured_testing', '')
        print("path ", pkg_path)
        lf_path = list(Path(pkg_path).rglob(f))   
        if not lf_path:
            raise Exception(f"Unable to find {f} in {pkg}")
        if len(lf_path) > 1:
            raise Exception(
                f"{f} found in multiple place, unsure what to pick {lf_path}"
            )
        file_paths.append(str(lf_path[0].resolve()))
    return file_paths


def getBagFiles(data):
    bag_files = data["bag_files"]
    file_paths = []
    bag_wd = os.getcwd()
    for b in bag_files:
        if not b.endswith(".db3"):
            raise Exception(f"{b} is not a bag file unable to use")
        interm_dir = '/src/structured_testing/test'
        complete_dir = bag_wd + interm_dir + b
        file_paths.append(str(Path(complete_dir).resolve()))
    return file_paths


def getAllPossibleObservers():
    res = set()
    for name, obj in inspect.getmembers(sys.modules[__name__]):
        if inspect.isclass(obj):
            non_observers = ["BaseObserver", "ObserverManager", "ObserverTypes"]
            if "Observer" in name and name not in non_observers:
                res.add(name)
    return res


def getObservers(data):
    observer_dicts = data["observers"]
    observers = []
    candidate_observers = getAllPossibleObservers()
    for obs in observer_dicts:
        if "observerClass" not in obs:
            raise Exception(
                "Defined Observer without class ensure observerClass present"
            )
        obsClassName = obs["observerClass"]
        if obsClassName not in candidate_observers:
            raise Exception(f"Observer of class {obsClassName} not found")
        try:
            clsmembers = inspect.getmembers(sys.modules[__name__], inspect.isclass)
            clsmembers = dict(clsmembers)
            obsClass = clsmembers[obsClassName]
            print('class ', obsClass)
            o = obsClass(**obs)
            observers.append(o)
        except Exception as e:
            print(f"Unable to Setup Observer: {obs}")
            raise e

    return observers


def getPublishedTopics(data):
    pub_topics = data["published_topics"]
    if not pub_topics:
        raise Exception("At least one topic to publish needed in .sim file")
    topics_set = set()
    for topic in pub_topics:
        topics_set.add(topic)
    return topics_set


def getRequiredTopics(data):
    req_topics = data["required_topics"]
    if not req_topics:
        raise Exception("At least one required topic needed in .sim file")
    topics_set = set()
    for topic in req_topics:
        topics_set.add(topic)
    return topics_set


def getTestDuration(data):
    duration = data["test_duration"][0]

    if not isinstance(duration, int):
        raise Exception("Invalid duration specified in .sim file. Must be integer")

    return duration


"""
Returns list of lists [all_launch_files], [all_bag_files], [observers], ...
[published_topics], [required_topics]
"""


def getYAMLData(yaml_file):
    with open(yaml_file, "r") as stream:
        data = yaml.safe_load(stream)
        lfs = getLaunchfiles(data)
        bags = getBagFiles(data)
        dur = getTestDuration(data)
        observers = getObservers(data)
        pub_topics = getPublishedTopics(data)
        req_topics = getRequiredTopics(data)
    return lfs, bags, dur, observers, pub_topics, req_topics


"""
Used to dump ordered dicts
"""


def dumpOrderedDict(data, filename):
    yaml_dump = yaml.dump(data, Dumper=OrderedDumper)
    with open(filename, "w") as f:
        f.write(yaml_dump)


class OrderedDumper(SafeDumper):
    def __init__(self, *args, **kwargs):
        super(OrderedDumper, self).__init__(*args, **kwargs)
        self.add_representer(OrderedDict, self.represent_dict_order)

    def represent_dict_order(self, tag, data):
        return self.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, data.items()
            )
