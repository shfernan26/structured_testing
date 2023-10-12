#!/usr/bin/env python
from .base_observer import BaseObserver


class InRangeObserver(BaseObserver):
    def __init__(
        self, *, name, topic, msgType, observerType, field, minVal, maxVal, **kwargs
    ):
        super().__init__(name, [topic], [msgType], observerType)
        self.field = field
        self.min = float(minVal)
        self.max = float(maxVal)

    def consumeMsg(self, topic, data):
        val = self.getField(data)
        if isinstance(val, list):
            temp_state = True
            for item in val:
                try:
                    subfield = item.dx
                    temp_state = temp_state and (subfield <= self.max and subfield >= self.min)
                except:
                    temp_state = False
                    print("Invalid subfield specified in try block of consumeMsg in InRangeObserver class.")
                    break

            current_state = temp_state
        
        else:
            current_state = val <= self.max and val >= self.min  
        
        
        self.logResult(current_state)

    def metadataDict(self):
        data = super().metadataDict()
        data["minValue"] = self.min
        data["maxValue"] = self.max
        return data
