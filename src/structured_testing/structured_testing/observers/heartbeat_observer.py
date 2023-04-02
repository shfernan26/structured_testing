#!/usr/bin/env python
from .base_observer import BaseObserver, ObserverTypes


class HeartbeatObserver(BaseObserver):
    def __init__(self, *, name, topic, msgType, **kwargs):
        super().__init__(name, [topic], [msgType], ObserverTypes.TRUE_ONCE)

    def consumeMsg(self, topic, data):
        self.logResult(True)  # Callback implies msg published
