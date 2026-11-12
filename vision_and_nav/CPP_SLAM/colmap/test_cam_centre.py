from pathlib import Path
import numpy as np
from viser.extras.colmap import read_images_binary

def qvec_to_rotmat(qvec):
    qw, qx, qy, qz = qvec
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [    2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw],
        [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy],
    ])

images = read_images_binary(Path("./sparse/0/images.bin"))

for image_id, img in images.items():
    R_cw = qvec_to_rotmat(img.qvec)
    t_cw = img.tvec
    C_w = -R_cw.T @ t_cw

    print("image", image_id)
    print("qvec =", img.qvec)
    print("t_cw =", t_cw)
    print("C_w  =", C_w)
