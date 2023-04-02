#!/usr/bin/env python
from .base_observer import BaseObserver, ObserverTypes

# import rospy
import rclpy


class FrequencyObserver(BaseObserver):
    def __init__(self, *, name, topic, msgType, minFreq, **kwargs):
        super().__init__(name, [topic], [msgType], ObserverTypes.TRUE_AT_END)
        self.node = rclpy.create_node(name+'_FreqObs')
        self.start_time = None
        self.most_recent_msg_time = None
        self.num_samples = 0
        self.minFreq = float(minFreq)
        self.type = ObserverTypes.TRUE_AT_END

    def consumeMsg(self, topic, data):
        clock = self.node.get_clock()
        if not self.start_time:
            self.start_time = clock.now()
        else:
            self.most_recent_msg_time = clock.now()
        self.num_samples += 1

    def getResult(self):
        if self.num_samples <= 1:  # Cannot infer freq from one or less msg
            self.overallResult = False
            return False
        delta_time = (self.most_recent_msg_time - self.start_time).nanoseconds / 1e9
        freq = 1 / (delta_time / self.num_samples)
        self.overallResult = freq >= self.minFreq
        return self.overallResult

    def metadataDict(self):
        data = super().metadataDict()
        data["minFreq"] = self.minFreq
        return data
