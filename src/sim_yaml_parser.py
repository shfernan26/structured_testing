import sys, inspect
import yaml
from yaml import SafeDumper
from rospkg import RosPack
from pathlib import Path
from pydoc import locate
from collections import OrderedDict
from observers import *


rp = RosPack()


def getLaunchfiles(data):
    lf_data = data["launch_files"]
    file_paths = []
    for lf in lf_data:
        pkg = lf["package"]
        f = lf["file"]
        if not f.endswith(".launch"):
            raise Exception(f"{f} is not a launch file unable to use")
        pkg_path = rp.get_path(pkg)
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
    for b in bag_files:
        if not b.endswith(".bag"):
            raise Exception(f"{b} is not a bag file unable to use")
        file_paths.append(str(Path(b).resolve()))
    return file_paths


def getAllPossibleObservers():
    res = set()
    for name, obj in inspect.getmembers(sys.modules[__name__]):
        if inspect.isclass(obj):
            non_observers = ["BaseObserver", "ObserverManager", "ObserverTypes"]
            if "Observer" in name and name not in non_observers:
                res.add(name)
    print(res)
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
            o = obsClass(**obs)
            observers.append(o)
        except Exception as e:
            print(f"Unable to Setup Observer: {obs}")
            raise e

    return observers


"""
Returns pair of lists [all_launch_files], [all_bag_files]
"""


def getYAMLData(yaml_file):
    with open(yaml_file, "r") as stream:
        data = yaml.safe_load(stream)
        lfs = getLaunchfiles(data)
        bags = getBagFiles(data)
        observers = getObservers(data)
    return lfs, bags, observers


"""
Used to dump ordered dicts
"""


class OrderedDumper(SafeDumper):
    def __init__(self, *args, **kwargs):
        super(OrderedDumper, self).__init__(*args, **kwargs)
        represent_dict_order = lambda self, data: self.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, data.items()
        )
        self.add_representer(OrderedDict, represent_dict_order)


def dumpOrderedDict(data, filename):
    yaml_dump = yaml.dump(data, Dumper=OrderedDumper)
    with open(filename, "w") as f:
        f.write(yaml_dump)


if __name__ == "__main__":
    data = getYAMLData(
        "/home/parallels/Development/UWAFT/src/structured_testing/test_files/example.sim"
    )
    # o = data[2][1]
    # print(o)
    # dumpOrderedDict([o.metadataDict()], "tmp.yaml")
    getAllPossibleObservers()
