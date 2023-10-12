#!/usr/bin/env python
from .base_observer import BaseObserver


class DeltaObserver(BaseObserver):
    def __init__(
        self, *, name, topic, msgType, observerType, field, maxDelta, **kwargs
    ):
        super().__init__(name, [topic], [msgType], observerType)
        self.maxDelta = float(maxDelta)
        self.last_val = None

    def consumeMsg(self, topic, data):
        current_state = None
        if not self.last_val:
            self.last_val = self.getField(data)
            self.logResult(True) # Ill defined result assumed to be true
            return
        val = self.getField(data)
        delta = abs(val - self.last_val)
        current_state = delta <= self.maxDelta
        self.logResult(current_state)
        self.last_val = val

    def metadataDict(self):
        data = super().metadataDict()
        data["isIncreasing"] = self.isIncreasing
        return data
