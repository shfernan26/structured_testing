#!/usr/bin/env python
from enum import Enum
import functools
from functools import partial
from pydoc import locate


from collections import OrderedDict

import rclpy
from rclpy.node import Node
# from genpy.message import Message # Not yet in ROS2 (see below for usage)
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ObserverTypes(Enum):
    ALWAYS_TRUE = 1
    TRUE_ONCE = 2
    TRUE_AT_END = 3
    TRUE_AT_START = 4


class BaseObserver:
    def __init__(self, name, topics, msgTypes, observerType, **kwargs):
        self.name = name
        if isinstance(topics[0], list) and isinstance(msgTypes[0], list):
            raise AttributeError("Redundant list created in observer super initializer.")
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

    def getField(self, msg, field_idx=None, *args):
        '''
        If multiple topics (and thus fields) required, add field_idx
        paramter to getField call in Observer class that specifies index of field to use.
        '''
        attr = self.field
        def _getattr(msg, attr):
                return getattr(msg, attr, *args)

        if type(attr) is list:
            return functools.reduce(_getattr, [msg] + attr[field_idx].split("."))
        else:
            return functools.reduce(_getattr, [msg] + attr.split("."))

    def metadataDict(self):
        """Get Attributes we care about in dict (to be logged for later)."""
        data = OrderedDict()
        data["name"] = self.name
        if self.overallResult is not None:
            data["status"] = "PASS" if self.overallResult else "FAIL"
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
        self.subscribers = {} # holds subscribers for multi-topic observers
        self.syncedCallBacks = {}
        self.topic_observer_map = {}
        self.multi_topic_key = ""
        self.sub_list = []
        super().__init__('observer_manager')

    """
    Sets up subscriber to observer
    """

    def addObserver(self, observer):
        self.observers[observer.name] = observer
        print(f"ADDING {observer.name}")
        topics = observer.topics
        if len(topics) == 1:
            topic = topics[0]
            if topic not in self.subscribers:
                print(f"Setting up subscriber to {topic}")
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

        elif len(topics) == 2:
            for topic in topics:
                if topic not in self.subscribers:
                    print(f"Multi topic observer. Setting up subscriber to {topic}")
                    observer_name = topic[1:]+observer.name
                    sub = Subscriber(
                        self,
                        observer.msg_types[topic],
                        topic
                    )
                    self.subscribers[observer_name] = sub
                    self.sub_list.append(sub)

                if not self.multi_topic_key: 
                    self.multi_topic_key+=topic
                else:
                    self.multi_topic_key = self.multi_topic_key+","+topic

            if self.multi_topic_key in self.topic_observer_map:
                self.topic_observer_map[self.multi_topic_key].append(observer)
            else:
                self.topic_observer_map[self.multi_topic_key] = [observer]

            # you can use ApproximateTimeSynchronizer if msgs dont have exactly the same timestamp        
            self.time_sync = ApproximateTimeSynchronizer(
                [Subscriber(self, observer.msg_types[topics[0]], topics[0]), Subscriber(self,  observer.msg_types[topics[1]], topics[1])],
                queue_size=100,
                slop=0.5  # defines the delay (in seconds) with which messages can be synchronized
            )
            self.time_sync.registerCallback(partial(self.syncedCallback, self.multi_topic_key))

        else:
            raise NotImplementedError("Observers requiring 3 or more topics not currently supported")


    def genericCallback(self, extra_data, sub_data):
        topic = extra_data["topic"]
        for observer in self.topic_observer_map[topic]:
            observer.consumeMsg(topic, sub_data)


    def syncedCallback(self, multi_topic_key, topic1_data, topic2_data): 
        topic_list = multi_topic_key.split(",")
        for observer in self.topic_observer_map[multi_topic_key]:
            observer.consumeMsg(topic_list[0], topic1_data)
            observer.consumeMsg(topic_list[1], topic2_data)
        

    def getResults(self):
        results = []
        for name, o in self.observers.items():
            o.getResult()
            results.append(o.metadataDict())
        return results
