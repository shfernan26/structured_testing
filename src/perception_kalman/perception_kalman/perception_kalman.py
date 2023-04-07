#!/usr/bin/env python
from __future__ import print_function, division
import rclpy
from rclpy.node import Node
from math import log, pi, exp
from numpy import diag, eye, zeros, dot, tile, array
from numpy.linalg import inv, det
from common.msg import ObjectDeletionMsg, FilteredObjectMsg
from common.msg import AssociatedObjectMsg
import time
from threading import Lock


# 1 = center lane, 2 = left lane, 3 = right lane
def determine_lane(dy):
    if dy < -2.5:  # negative is left
        return 2, False
    elif dy > 2.5:  # positive is right
        return 3, False
    else:
        return 1, True


class MyKF():
    def __init__(self, initial_X, initial_Y, H, A):
        self.X = initial_X
        self.Y = initial_Y
        self.P = diag((0.01, 0.01, 0.01, 0.01))
        self.Q = eye(self.X.shape[0])
        self.B = eye(self.X.shape[0])
        self.U = zeros((self.X.shape[0], 1))
        self.H = H
        self.A = A
        self.R = eye(self.Y.shape[0])

    def kf_predict(self):
        self.X = dot(self.A, self.X) + dot(self.B, self.U)
        self.P = dot(self.A, dot(self.P, self.A.T)) + self.Q

    def gauss_pdf(self, X, M, S):
        if M.shape[1] == 1:
            DX = X - tile(M, X.shape[1])
            E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(det(S))
            P = exp(-E)
        elif X.shape[1] == 1:
            DX = tile(X, M.shape[1]) - M
            E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(det(S))
            P = exp(-E)
        else:
            DX = X-M
            E = 0.5 * dot(DX.T, dot(inv(S), DX))
            E = E + 0.5 * M.shape[0] * log(2 * pi) + 0.5 * log(det(S))
            P = exp(-E)
        return (P[0], E[0])

    def kf_update(self, Y):
        self.Y = Y
        IM = dot(self.H, self.X)
        IS = self.R + dot(self.H, dot(self.P, self.H.T))
        K = dot(self.P, dot(self.H.T, inv(IS)))
        self.X = self.X + dot(K, (Y-IM))
        self.P = self.P - dot(K, dot(IS, K.T))
        # LH = self.gauss_pdf(Y, IM, IS)


class KF_Node(Node):
    def __init__(self):
        super().__init__("perception_kalman")
        self.objects = {}
        self.create_subscription(AssociatedObjectMsg, 'associated_object',
                                 callback=self.obj_association_callback, qos_profile=10)
        self.create_subscription(ObjectDeletionMsg, 'obj_deletion',
                                 callback=self.object_deletion_callback, qos_profile=10)
        self.output = self.create_publisher(FilteredObjectMsg, 'filtered_obj', qos_profile=10)
        self.resource_lock = Lock()

        self.start = time.time()

    def obj_association_callback(self, obj):
        self.resource_lock.acquire()
        self.get_logger().info('sensor fused callback called %d @ %f, %f'
                               % (obj.obj_id, obj.obj_dx, obj.obj_dy))

        measurement = [[obj.obj_dx],
                       [obj.obj_dy],
                       [obj.obj_vx],
                       [obj.obj_vy]]

        if obj.obj_id not in self.objects:
            self.get_logger().info('creating object %d' % obj.obj_id)
            dt = 0.005
            # Initialization of state matrices
            X = array(measurement)
            A = array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
            H = array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            self.objects[obj.obj_id] = MyKF(X, array(measurement), H, A)

        elif self.objects[obj.obj_id].Y[0, 0] != measurement[0][0] or \
                self.objects[obj.obj_id].Y[1, 0] != measurement[1][0] or \
                self.objects[obj.obj_id].Y[2, 0] != measurement[2][0] or \
                self.objects[obj.obj_id].Y[3, 0] != measurement[3][0]:
            hashed = self.objects[obj.obj_id]
            hashed.kf_predict()
            hashed.kf_update(array(measurement))

            if hashed.X[0] > 255:
                hashed.X[0] = 255
            if hashed.X[0] < 0:
                hashed.X[0] = 0
            if hashed.X[1] > 128:
                hashed.X[1] = 128
            if hashed.X[1] < -128:
                hashed.X[1] = -128

        result = FilteredObjectMsg()
        result.obj_dx = self.objects[obj.obj_id].X[0, 0]
        result.obj_dy = self.objects[obj.obj_id].X[1, 0]
        result.obj_vx = self.objects[obj.obj_id].X[2, 0]
        result.obj_id = obj.obj_id
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        result.obj_timestamp = obj.obj_timestamp
        result.obj_count = 30
        self.output.publish(result)
        self.resource_lock.release()

    def object_deletion_callback(self, obj):
        del self.objects[obj.obj_id]


def main():
    rclpy.init()
    node = KF_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
