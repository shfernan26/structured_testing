#!/usr/bin/python3
from .base_observer import BaseObserver, ObserverTypes


class MinMaxObserver(BaseObserver):
    def __init__(
        self, *, name, topic, msgType, observerType, field, isMin, value, **kwargs
    ):
        super().__init__(name, [topic], [msgType], observerType)
        self.isMin = isMin
        self.value = float(value)
        self.field = field

    def consumeMsg(self, topic, data):
        current_state = None
        val = float(self.getField(data))
        if self.isMin:
            current_state = val > self.value
        else:
            current_state = val < self.value
        self.logResult(current_state)

    def metadataDict(self):
        data = super().metadataDict()
        data["isMin"] = self.isMin
        data["value"] = self.value
        return data
