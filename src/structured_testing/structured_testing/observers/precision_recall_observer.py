#!/usr/bin/env python
from .base_observer import BaseObserver, ObserverTypes

# import rospy
import rclpy
import numpy as np


class PrecisionRecallObserver(BaseObserver):
    def __init__(self, *, name, topic, msgType, field, num_targets, output_type, tol, precision_pass_score, recall_pass_score, **kwargs):
        super().__init__(name, topic, msgType, ObserverTypes.TRUE_AT_END) 
        self.node = rclpy.create_node(name+'_PrecRecallObs')
        self.topics = topic
        self.field = field
        self.num_targets = num_targets
        self.output_type = output_type
        self.tol = tol
        self.precision_pass_score = precision_pass_score
        self.recall_pass_score = recall_pass_score
        self.type = ObserverTypes.TRUE_AT_END
        self.gt_values = []
        self.detections = []
        self.zero_tol = 0.1 # detections less than this are considred
        self.precision = 0.0
        self.recall = 0.0


    def consumeMsg(self, topic, data):
        # Detections
        if topic == self.topics[0]:
            dx = self.getField(data, field_idx=0)
            dy = self.getField(data, field_idx=1)
            det_at_time = []
            for i in range(len(dx)):
                det_at_time.append([dx[i], dy[i]])
            self.detections.append(det_at_time)
            
        # Ground Truth
        if topic == self.topics[1]:
            dx = self.getField(data, field_idx=2)
            dy = self.getField(data, field_idx=3)
            gt_at_time = []
            for i in range(self.num_targets):
                gt_at_time.append([dx[i], dy[i]])
            self.gt_values.append(gt_at_time)


    def getResult(self):

        if len(self.gt_values) != len(self.detections):
            raise ValueError("Length of ground truth and detections arrays do not match")

        TP = 0.0
        FP = 0.0
        FN = 0.0
        
        for time_step in range(len(self.gt_values)):
            # Find which detections is closest to each ground truth object
            for gt_obj in self.gt_values[time_step]:
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
            
        try:     
            self.precision = TP / (TP + FP)
        except ZeroDivisionError:
            self.precision = float('-inf')

        try:
            self.recall = TP / (TP+FN)
        except ZeroDivisionError:
            self.recall = float('-inf')


        if (self.precision >= self.precision_pass_score) and (self.recall >= self.recall_pass_score):
            self.overallResult = True
        else:
            self.overallResult = False
        return self.overallResult

    
    def euclidean_distance(self, gt_coord, det_coord): 
        return np.sqrt((gt_coord[0] - det_coord[0])**2 + (gt_coord[1] - det_coord[1])**2)


    def metadataDict(self):
        data = super().metadataDict()
        data["tol"] = self.tol
        data["precisionPassScore"] = self.precision_pass_score
        data["recallPassScore"] = self.recall_pass_score
        data["precision"] = round(self.precision, 2)
        data["recall"] = round(self.recall,2)
        return data
