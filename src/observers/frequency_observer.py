#!/usr/bin/python3
from .base_observer import BaseObserver, ObserverTypes

import rospy


class FrequencyObserver(BaseObserver):
    def __init__(self, name, *, topic, msgType, minFreq, **kwargs):
        super().__init__(name, [topic], [msgType], ObserverTypes.TRUE_AT_END)
        self.start_time = None
        self.most_recent_msg_time = None
        self.num_samples = 0
        self.minFreq = float(minFreq)
        self.type = ObserverTypes.TRUE_AT_END

    def consumeMsg(self, topic, data):
        if not self.start_time:
            self.start_time = rospy.Time.now()
        else:
            self.most_recent_msg_time = rospy.Time.now()
        self.num_samples += 1

    def getResult(self):
        if self.num_samples <= 1:  # Cannot infer freq from one or less msg
            self.overallResult = False
            return False
        delta_time = self.most_recent_msg_time - self.start_time
        freq = 1 / (delta_time.to_sec() / self.num_samples)
        self.overallResult = freq >= self.minFreq
        return self.overallResult

    def metadataDict(self):
        data = super().metadataDict()
        data["minFreq"] = self.minFreq
        return data
