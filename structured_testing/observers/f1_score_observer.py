#!/usr/bin/env python
from .base_observer import BaseObserver, ObserverTypes

# import rospy
import rclpy
import numpy as np


class F1ScoreObserver(BaseObserver):
    def __init__(self, *, name, topic, msgType, field, num_targets, output_type, tol, f1_pass_score, **kwargs):
        super().__init__(name, topic, msgType, ObserverTypes.TRUE_AT_END) 
        self.topics = topic
        self.field = field
        self.num_targets = num_targets
        self.output_type = output_type
        self.tol = tol
        self.f1_pass_score = f1_pass_score
        self.type = ObserverTypes.TRUE_AT_END
        self.gt_values = []
        self.detections = []
        self.zero_tol = 0.1 # detections less than this are considred
        self.precision = 0.0
        self.recall = 0.0
        self.f_score = 0.0


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

        TP = 0.0
        FP = 0.0
        FN = 0.0
        
        for time_step in range(len(self.gt_values)):
            # Find which detections is closest to each ground truth object (if detections exist)
            for gt_obj in self.gt_values[time_step]:
                if len(self.detections[time_step]) > 0:
                    dist_list = []
                    for target in self.detections[time_step]:
                        dist = self.euclidean_distance(gt_obj, target)
                        dist_list.append(dist)

                    closest_target_idx = min(range(len(dist_list)), key=dist_list.__getitem__)

                    # If detection within tolerance, count it as true positive, else it is false positive
                    if self.euclidean_distance(gt_obj, self.detections[time_step][closest_target_idx]) < self.tol:
                        TP+=1
                    else:
                        FP+=1

                    # Replace closest detection with inf so it won't be used again
                    self.detections[time_step][closest_target_idx] = [float("inf"), float("inf")]

            # If more detections than ground truth, add that difference to false positives
            # If less detections, add it to false negatives
            if self.output_type == "variable":
                num_detections = len(self.detections[time_step])
                if  num_detections > self.num_targets:
                    FP += (num_detections - self.num_targets)

                elif num_detections < self.num_targets:
                    FN +=  (self.num_targets - num_detections)
                    
                else:
                    pass

            elif self.output_type == "fixed":
                num_detections = sum((sum(1 for coord in coord_pair if coord >0)) > 0 for coord_pair in self.detections[time_step])
                if  num_detections > self.num_targets:
                    FP += (num_detections - self.num_targets)

                elif num_detections < self.num_targets:
                    FN += (self.num_targets - num_detections)

                else:
                    pass
            
            else:
                raise AttributeError("Invalid output type specified")
        
        # print(f"TP {TP}, FP {FP}, FN {FN}")
            
        try:     
            self.precision = TP / (TP + FP)
        except ZeroDivisionError:
            self.precision = float('-inf')

        try:
            self.recall = TP / (TP+FN)
        except ZeroDivisionError:
            self.recall = float('-inf')

        try:
            self.f_score =  2*((self.precision*self.recall) / (self.precision + self.recall))
        except ZeroDivisionError:
            self.f_score = float('-inf')

        # print(f"Precision: {self.precision}, Recall: {self.recall}, F Score: {self.f_score}")


        if (self.f_score >= self.f1_pass_score):
            self.overallResult = True
        else:
            self.overallResult = False
        return self.overallResult

    
    def euclidean_distance(self, gt_coord, det_coord): 
        return np.sqrt((gt_coord[0] - det_coord[0])**2 + (gt_coord[1] - det_coord[1])**2)


    def metadataDict(self):
        data = super().metadataDict()
        data["tol"] = self.tol
        data["f1PassScore"] = self.f1_pass_score
        data["precision"] = round(self.precision, 2)
        data["recall"] = round(self.recall,2)
        data["fScore"] = round(self.f_score,2)
        return data
