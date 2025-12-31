def invert_homogeneous(T):
    R = [row[:3] for row in T[:3]]
    t = [row[3] for row in T[:3]]

    R_inv = [[R[j][i] for j in range(3)] for i in range(3)]

    t_inv = []
    for i in range(3):
        s = 0.0
        for k in range(3):
            s += R_inv[i][k] * t[k]
        t_inv.append(-s)

    T_inv = [
        [R_inv[0][0], R_inv[0][1], R_inv[0][2], t_inv[0]],
        [R_inv[1][0], R_inv[1][1], R_inv[1][2], t_inv[1]],
        [R_inv[2][0], R_inv[2][1], R_inv[2][2], t_inv[2]],
        [0.0,         0.0,         0.0,         1.0]
    ]
    return T_inv

## Put the T_cam_imu outputed from calibration here ################
T_cam_imu = [
    [-0.0012, -0.9999,  0.0089,  0.0237],
    [ 0.0012, -0.0089, -0.9999,  0.0019],
    [ 0.9999, -0.0011,  0.0012, -0.0049],
    [ 0.0,     0.0,     0.0,     1.0   ]
]
#####################################################################

T_imu_cam = invert_homogeneous(T_cam_imu)

for row in T_imu_cam:
    print(row)