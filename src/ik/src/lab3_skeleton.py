#!/usr/bin/env python
"""
Lab 3, task 1
"""


import numpy as np
#import scipy as sp
import kin_func_skeleton as kfs

def lab3(thetas):
    q = np.ndarray((3,8))
    w = np.ndarray((3,7))
   
    q[0:3,0] = [0.0635, 0.2598, 0.1188]
    q[0:3,1] = [0.1106, 0.3116, 0.3885]
    q[0:3,2] = [0.1827, 0.3838, 0.3881]
    q[0:3,3] = [0.3682, 0.5684, 0.3181]
    q[0:3,4] = [0.4417, 0.6420, 0.3177]
    q[0:3,5] = [0.6332, 0.8337, 0.3067]
    q[0:3,6] = [0.7152, 0.9158, 0.3063]
    q[0:3,7] = [0.7957, 0.9965, 0.3058]

    w[0:3,0] = [-0.0059,  0.0113,  0.9999]
    w[0:3,1] = [-0.7077,  0.7065, -0.0122]
    w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,3] = [-0.7077,  0.7065, -0.0122]
    w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,5] = [-0.7077,  0.7065, -0.0122]
    w[0:3,6] = [ 0.7065,  0.7077, -0.0038]



    w0xq0 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][0], w[1][0], w[2][0]])), np.array([q[0][0], q[1][0], q[2][0]]))
    xi_0 = [w0xq0[0], w0xq0[1], w0xq0[2], w[0][0], w[1][0], w[2][0]]
    w1xq1 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][1], w[1][1], w[2][1]])), np.array([q[0][1], q[1][1], q[2][1]]))
    xi_1 = [w1xq1[0], w1xq1[1], w1xq1[2], w[0][1], w[1][1], w[2][1]]
    w2xq2 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][2], w[1][2], w[2][2]])), np.array([q[0][2], q[1][2], q[2][2]]))
    xi_2 = [w2xq2[0], w2xq2[1], w2xq2[2], w[0][2], w[1][1], w[2][2]]
    w3xq3 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][3], w[1][3], w[2][3]])), np.array([q[0][3], q[1][3], q[2][3]]))
    xi_3 = [w3xq3[0], w3xq3[1], w3xq3[2], w[0][3], w[1][1], w[2][3]]
    w4xq4 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][4], w[1][4], w[2][4]])), np.array([q[0][4], q[1][4], q[2][4]]))
    xi_4 = [w4xq4[0], w4xq4[1], w4xq4[2], w[0][4], w[1][1], w[2][4]]
    w5xq5 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][5], w[1][5], w[2][5]])), np.array([q[0][5], q[1][5], q[2][5]]))
    xi_5 = [w5xq5[0], w5xq5[1], w5xq5[2], w[0][5], w[1][1], w[2][5]]
    w6xq6 = -1 * np.matmul(kfs.skew_3d(np.array([w[0][6], w[1][6], w[2][6]])), np.array([q[0][6], q[1][6], q[2][6]]))
    xi_6 = [w6xq6[0], w6xq6[1], w6xq6[2], w[0][6], w[1][1], w[2][6]]

    xi_array = np.array([xi_0, xi_1, xi_2, xi_3, xi_4, xi_5, xi_6], dtype=np.float64).T

    R = np.array([[0.0076, 0.0001, -1.0000],
                          [-0.7040, 0.7102, -0.0053],
                          [0.7102, 0.7040, 0.0055]]).T

    gOfZero = np.array([[R[0][0], R[0][1], R[0][2], q[0][7]],
                        [R[1][0], R[1][1], R[1][2], q[1][7]],
                        [R[2][0], R[2][1], R[2][2], q[2][7]],
                        [0, 0, 0, 1]])

    g = np.matmul(kfs.prod_exp(xi_array, thetas), gOfZero)

    return g


if __name__ == "__main__":
    print('Lab 3')
    lab3()