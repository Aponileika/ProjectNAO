import numpy as np
#Camera matrix for Jonathans webcam, calibration is ok (RMSE of reprojection ~1pixel)
K_JW = np.array([
    [9.747187409387847765e+02, 0.000000000000000000e+00, 6.663249058750432141e+02],
    [0.000000000000000000e+00, 9.765223334221673213e+02, 3.374737864029501111e+02],
    [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
                 ], dtype=np.float32)
K_JW.flags.writeable = False
dist_coeffs = np.array([6.475901025911835751e-02,
                        -1.903655376657792664e-01,
                        -3.666863513699757018e-03,
                        2.119531347424837616e-03,
                        1.113497353924944727e-01])
dist_coeffs.flags.writeable = False
