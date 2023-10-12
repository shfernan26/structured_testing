#!/usr/bin/env python
from .base_observer import BaseObserver, ObserverTypes

import rclpy


class TemplateObserver(BaseObserver):


    '''
    Add any additional fields required by the Observer 
    '''
    def __init__(self, *, name, topic, msgType, observerType, **kwargs): # 
        
        super().__init__(name, [topic], [msgType], observerType)

        
    def consumeMsg(self, topic, data):
        '''
        Do something when data from specific topic is received.
        Args:
            topic: ROS topic name of interest. If multiple topics, topic of interest can 
                be accessed by indexing (e.g. topic[1]). See F1 Score Observer for example.
            data: ROS 2 data associated with specified topic.
        '''


    def getResult(self):
        '''
        Sets the final pass/fail value of the Observer after the simulation is over. 
            Overloads Base Observer getResult(). See Frequency Observer for example.
        Returns:
            Boolean pass/fail value.
        '''
    

    def metadataDict(self):
        '''
        Adds additional output to the standard dictionary that is printed to .simresults file. 
            Overloads Base Observer metadataDict(). See Frequency Observer for example.
        Returns:
            Dict of Observer output data.
        '''
        
