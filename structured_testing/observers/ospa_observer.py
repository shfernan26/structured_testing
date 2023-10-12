#!/usr/bin/env python
from .base_observer import BaseObserver, ObserverTypes

# import rospy
import rclpy
import numpy as np
from statistics import mean
from observers.calc_ospa import ospa


class OSPAObserver(BaseObserver):
    def __init__(self, *, name, topic, msgType, field, num_targets, cut_off, sens, ospa_pass_score, **kwargs):
        super().__init__(name, topic, msgType, ObserverTypes.TRUE_AT_END) 
        self.topics = topic
        self.field = field
        self.num_targets = num_targets
        self.cut_off = cut_off
        self.sens = sens
        self.ospa_pass_score = ospa_pass_score
        self.type = ObserverTypes.TRUE_AT_END
        self.gt_values = []
        self.detections = []
        self.mean_ospa = 0.0
        

    def consumeMsg(self, topic, data):
        # Detections
        if topic == self.topics[0]:
            det_at_time = []
            contents = self.getField(data, field_idx=0) 
            for i in range(len(contents)):
                dx = contents[i].dx
                dy = contents[i].dy
                det_at_time.append([dx, dy])
            self.detections.append(det_at_time)
            
        # Ground Truth
        if topic == self.topics[1]:
            contents = self.getField(data, field_idx=1)
            gt_at_time = []
            for i in range(self.num_targets):
                dx = contents[i].dx
                dy = contents[i].dy
                gt_at_time.append([dx, dy])
            self.gt_values.append(gt_at_time)


    def getResult(self):
        if len(self.gt_values) != len(self.detections):
            raise ValueError("Length of ground truth and detections arrays do not match")

        ospa_lst = []
        
        for time_step in range(len(self.gt_values)):
            gt_lst = []
            curr_gt_row = self.gt_values[time_step]
            for i in range(0, len(curr_gt_row)):
                det_pair = np.array(curr_gt_row[i])
                gt_lst.append(det_pair)

            det_lst = []
            curr_det_row = self.detections[time_step]
            for i in range(0, len(curr_det_row)):
                det_pair = np.array(curr_det_row[i])
                det_lst.append(det_pair)

            ospa_metric = ospa(X=det_lst, Y = gt_lst, c=self.cut_off, p=self.sens)
            ospa_lst.append(ospa_metric)
        
        self.mean_ospa = float(mean(ospa_lst))

        # print(f"OSPA: {self.mean_ospa}")


        if (self.mean_ospa <= self.ospa_pass_score):
            self.overallResult = True
        else:
            self.overallResult = False
        return self.overallResult



    def metadataDict(self):
        data = super().metadataDict()
        data["cutoff"] = self.cut_off
        data["sensitivity"] = self.sens
        data["ospaPassScore"] = self.ospa_pass_score
        data["meanOSPA"] = round(self.mean_ospa ,2)
        return data
