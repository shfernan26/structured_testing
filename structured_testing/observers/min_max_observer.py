#!/usr/bin/env python
from .base_observer import BaseObserver


class MinMaxObserver(BaseObserver):
    def __init__(self, *, name, topic, msgType, observerType, field, isMin, value, **kwargs):
        super().__init__(name, [topic], [msgType], observerType)
        self.isMin = isMin
        self.value = float(value)
        self.field = field

        if self.isMin:
            self.actualValue = float('inf')
        else:
            self.actualValue = float('-inf')

    def consumeMsg(self, topic, data):
        current_state = None
        val = self.getField(data)

        if isinstance(val, list):
            temp_state = True
            for item in val:
                try:
                    subfield = item.dx
                    if self.isMin:
                        temp_state = temp_state and (subfield >= self.value)
                        self.actualValue = min(self.actualValue, subfield)
                    else:
                        temp_state = temp_state and (subfield <= self.value)
                        self.actualValue = max(self.actualValue, subfield)

                except:
                    temp_state = False
                    print("Invalid subfield specified. Change MinMax Obs class L21 to alternate field")
                    break

            current_state = temp_state
        
        else:
            if self.isMin:
                current_state = float(val) >= self.value
                self.actualValue = min(self.actualValue, float(val))
            else:
                current_state = float(val) <= self.value
                self.actualValue = max(self.actualValue, float(val))

        self.logResult(current_state)

    def metadataDict(self):
        data = super().metadataDict()
        data["isMin"] = self.isMin
        data["targetValue"] = self.value
        data["actualValue"] = self.actualValue
        return data
