# ***************************************************************************************/
# *    This code is based on the original work cited below. Modifications have been made 
# *    to suit our project purposes
# *
# *    Title: OSPA
# *    Author:  djape24394
# *    Date: 2021
# *    Availability: https://github.com/djape24394/gmphd_filter/blob/master/ospa.py
# *
# ***************************************************************************************/

from scipy.optimize import linear_sum_assignment
import numpy as np
import numpy.linalg as lin

import pandas as pd
from statistics import mean

def _calculation(X, Y, c, p):
    def d_c(x, y, c):
        return min(c, lin.norm(x - y))

    m = len(X)
    n = len(Y)
    if m == 0 and n == 0:
        return 0, 0, 0

    if m > n:
        # swap
        X, Y = Y, X
        m, n = n, m

    card_dist = c ** p * (n - m)

    D = np.zeros((n, n))
    for i in range(m):
        for j in range(n):
            D[i, j] = d_c(X[i], Y[j], c) ** p
    D[m:, :] = c ** p
    row_ind, col_ind = linear_sum_assignment(D)
    local_dist = D[row_ind[:m], col_ind[:m]].sum()

    return local_dist, card_dist, n


def ospa(X, Y, c, p):
    """
    Calculates Optimal Subpattern Assignment (OSPA) metric, defined by Dominic Schuhmacher, Ba-Tuong Vo, and Ba-Ngu Vo
    in "A Consistent Metric for Performance Evaluation of Multi-Object Filters". This is implementation using Hungarian
    method.
    https://en.wikipedia.org/wiki/Hungarian_algorithm
    http://www.hungarianalgorithm.com/examplehungarianalgorithm.php
    :param X: set of ndarray vectors
    :param Y: set of ndarray vectors
    :param c: c>0 . "The cut-off parameter c determines the relative weighting of the penalties assigned to
    cardinality and localization errors. A value of c which corresponds to the magnitude
    of a typical localization error can be considered small and has the effect of emphasizing
    localization errors. A value of c which corresponds to the maximal distance between
    targets can be considered large and has the effect of emphasizing cardinality errors."
    from Bayesian Multiple Target Filtering Using Random Finite Sets, BA-NGU VO, BA-TUONG VO, AND DANIEL CLARK
    :param p: The order parameter p determines the sensitivity of the metric to outliers. p>=1
    :return:
    """
    local_dist, card_dist, n = _calculation(X, Y, c, p)
    if n == 0:
        return 0
    return (1 / n * (local_dist + card_dist)) ** (1 / p)


def ospa_local_card(X, Y, c, p):
    local_dist, card_dist, n = _calculation(X, Y, c, p)
    if n == 0:
        return 0, 0
    return (1 / n * local_dist) ** (1 / p), (1 / n * card_dist) ** (1 / p)


def ospa_all(X, Y, c, p):
    local_dist, card_dist, n = _calculation(X, Y, c, p)
    if n == 0:
        return 0, 0, 0
    return (1 / n * (local_dist + card_dist)) ** (1 / p), (1 / n * local_dist) ** (1 / p), (1 / n * card_dist) ** (
            1 / p)


'''
Following code is an example implementation of ospa function above
'''

def main():
    fcm_df = pd.read_csv('test_data/scenario1_vehicles-further-apart/stock_front_cam_output.csv')
    lrr_df = pd.read_csv('test_data/scenario1_vehicles-further-apart/stock_long_range_radar_output.csv')
    gt_df = pd.read_csv('test_data/scenario1_vehicles-further-apart/ground_truth_output.csv')

    gt_hdr_list = []
    num_gt_objects = 2
    for i in range(1,num_gt_objects+1):
        dx_header = "Detected Dx_"+str(i)
        gt_hdr_list.append(dx_header)
        dy_header = "Detected Dy_"+str(i)
        gt_hdr_list.append(dy_header)
    gt_df = gt_df[gt_hdr_list]

    det_hdr_list = []
    num_det_objects = 10
    for i in range(1,num_det_objects+1):
        dx_header = "Detected Dx_"+str(i)
        det_hdr_list.append(dx_header)
        dy_header = "Detected Dy_"+str(i)
        det_hdr_list.append(dy_header)
    fcm_df = fcm_df[det_hdr_list]


    ospa_lst = []
    # make sure indexes pair with number of rows and drop index col
    gt_df = gt_df.reset_index(drop=True) 
    fcm_df = fcm_df.reset_index(drop=True)

    for row_idx in range(1,len(gt_df.index)):
        gt_lst = []
        curr_gt_row = gt_df.iloc[row_idx]
        for i in range(0, len(curr_gt_row), 2):
            det_pair = np.array([curr_gt_row[i], curr_gt_row[i+1]])
            gt_lst.append(det_pair)

        det_lst = []
        curr_det_row = fcm_df.iloc[row_idx]
        for i in range(0, len(curr_det_row), 2):
            det_pair = np.array([curr_det_row[i], curr_det_row[i+1]])
            det_lst.append(det_pair)
    
        ospa_metric = ospa(X=det_lst, Y = gt_lst, c=5, p=1)
        ospa_lst.append(ospa_metric)

    print(ospa_lst)
    print(mean(ospa_lst))

if __name__ == "__main__":
    main()