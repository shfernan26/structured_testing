#!/usr/bin/env python
from enum import Enum
import functools
from functools import partial
from pydoc import locate


from collections import OrderedDict

import rclpy
from rclpy.node import Node
# from genpy.message import Message # Not yet in ROS2 (see below for usage)


class ObserverTypes(Enum):
    ALWAYS_TRUE = 1
    TRUE_ONCE = 2
    TRUE_AT_END = 3
    TRUE_AT_START = 4


class BaseObserver:
    def __init__(self, name, topics, msgTypes, observerType, **kwargs):
        self.name = name
        self.topics = topics
        self.field = None
        if type(observerType) != ObserverTypes:
            type(observerType)
            try:
                observerType = ObserverTypes[observerType]
            except Exception:
                raise Exception(
                    f"Unable to convert {observerType} to ObserverType Enum object"
                )

        self.type = observerType
        self.overallResult = None
        self.intermediateResult = True
        self.msg_types = {}
        self.results = []
        if len(msgTypes) != len(topics):
            raise Exception(
                f"Number of types given {len(msgTypes)} is not equivalent to \
                number of types: {len(topics)}"
            )
        for i, t in enumerate(msgTypes):
            msg_type = locate(t)
            if not msg_type:
                raise Exception(
                    f"Type {t} not found, ensure it has been added to the ROS_PATH"
                )
            # genpy (ROS1) equilavent not yet found in ROS2...
            # if not issubclass(msg_type, Message):
            #     raise Exception(f"Type {msg_type} is not a genpy msg")
            self.msg_types[topics[i]] = msg_type
        if self.type == ObserverTypes.TRUE_ONCE:
            self.intermediateResult = False

    def logResult(self, res):
        """Intended to be called throughout sim to log result and enforce types."""
        # print("observer class ", self.__class__.__name__)
        # print("log result ", res)
        if self.overallResult:  # Result is determined current result does not matter
            return
        if self.type == ObserverTypes.ALWAYS_TRUE:
            self.intermediateResult &= res
            if not res:
                self.overallResult = False
        if self.type == ObserverTypes.TRUE_ONCE:
            self.intermediateResult |= res
        if self.type == ObserverTypes.TRUE_AT_END:
            self.intermediateResult = res
        if self.type == ObserverTypes.TRUE_AT_START:
            self.overallResult = res

    def getResult(self):
        if not self.overallResult:
            self.overallResult = self.intermediateResult
        return self.overallResult

    def consumeMsg(self, topic, data):
        pass

    def getField(self, msg, *args):
        attr = self.field

        def _getattr(msg, attr):
            return getattr(msg, attr, *args)

        return functools.reduce(_getattr, [msg] + attr.split("."))

    def metadataDict(self):
        """Get Attributes we care about in dict (to be logged for later)."""
        data = OrderedDict()
        data["name"] = self.name
        if self.overallResult is not None:
            data["Status"] = "PASS" if self.overallResult else "FAIL"
        else:
            data["status"] = "Undetermined"
        data["observerClass"] = self.__class__.__name__
        data["topic(s)"] = self.topics
        if self.field:
            data["field"] = self.field
        data["observerType"] = self.type.name
        return data

    def __str__(self):
        return f"{__class__}: {self.name}"


class ObserverManager(Node):
    def __init__(self):
        self.observers = {}
        self.subscribers = {}
        self.syncedCallBacks = {}
        self.topic_observer_map = {}

    """
    Sets up subscriber to observer
    """

    def addObserver(self, observer):
        self.observers[observer.name] = observer
        topics = observer.topics
        if len(topics) == 1:
            topic = topics[0]
            if topic not in self.subscribers:
                print(f"Setting up subscriber to {topic}")
                try:
                    rclpy.init()
                except Exception:
                    if not rclpy.ok():
                        print("Init failure")
                observer_name = topic[1:]+"_observer"
                super().__init__(observer_name)
                self.create_subscription(
                    observer.msg_types[topic],
                    topic,
                    callback=partial(self.genericCallback, {"topic": topic}),
                    qos_profile=10
                )

            if topic in self.topic_observer_map:
                self.topic_observer_map[topic].append(observer)
            else:
                self.topic_observer_map[topic] = [observer]

        else:
            # TODO: When an observer has multiple topics need to setup a time synchronizer
            raise Exception("Observers with multiple topics are not supported yet")

    def genericCallback(self, extra_data, sub_data):
        topic = extra_data["topic"]
        for observer in self.topic_observer_map[topic]:
            observer.consumeMsg(topic, sub_data)

    def syncedCallback(self, *args):
        # TODO: For observers that have multiple topics
        pass

    def getResults(self):
        results = []
        for name, o in self.observers.items():
            o.getResult()
            results.append(o.metadataDict())
        return results
