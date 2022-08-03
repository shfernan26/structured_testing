#!/usr/bin/python3
from .base_observer import BaseObserver, ObserverTypes


class MonotonicObserver(BaseObserver):
    def __init__(
        self, *, name, topic, msgType, observerType, field, isIncreasing, **kwargs
    ):
        super().__init__(name, [topic], [msgType], observerType)
        self.isIncreasing = bool(isIncreasing)
        self.last_val = float("inf")
        if self.isIncreasing:
            self.last_val *= -1  # IF need increasing start at -inf
        self.field = field

    def consumeMsg(self, topic, data):
        current_state = None
        val = self.getField(data)
        if self.isIncreasing:
            current_state = val >= self.last_val
        else:
            current_state = val <= self.last_val
        self.logResult(current_state)

    def metadataDict(self):
        data = super().metadataDict()
        data["isIncreasing"] = self.isIncreasing
        return data
