from pathlib import Path
import time

import imageio.v3 as iio
import numpy as np
import viser
import viser.transforms as vtf

from viser.extras.colmap import (
    read_cameras_binary,
    read_images_binary,
    read_points3d_binary,
)


def main(root: str):
    root = Path(root)

    sparse_path = root / "sparse" / "0"
    images_path = root / "images"
    print(f"sparse path = {sparse_path}");
    print(f"images path = {images_path}");

    cameras = read_cameras_binary(sparse_path / "cameras.bin")
    images = read_images_binary(sparse_path / "images.bin")
    points3d = read_points3d_binary(sparse_path / "points3D.bin")

    server = viser.ViserServer()

    # -------------------------
    # Sparse point cloud
    # -------------------------
    points = np.array([p.xyz for p in points3d.values()])
    colors = np.array([p.rgb for p in points3d.values()])

    server.scene.add_point_cloud(
        name="/colmap/points",
        points=points,
        colors=colors,
        point_size=0.02,
    )

    # -------------------------
    # Cameras / frustums
    # -------------------------
    for img_id, img in images.items():
        cam = cameras[img.camera_id]

        # COLMAP stores world-to-camera:
        #   x_cam = R * x_world + t
        #
        # For visualization we usually want camera-to-world,
        # so invert it.
        T_world_camera = vtf.SE3.from_rotation_and_translation(
            vtf.SO3(img.qvec),   # qvec is [qw, qx, qy, qz]
            img.tvec,
        ).inverse()

        frame = server.scene.add_frame(
            f"/colmap/frame_{img_id}",
            wxyz=T_world_camera.rotation().wxyz,
            position=T_world_camera.translation(),
            axes_length=0.1,
            axes_radius=0.005,
        )

        image_file = images_path / img.name

        image = None
        if image_file.exists():
            image = iio.imread(image_file)

        # Viser example assumes PINHOLE:
        # cam.params = (fx, fy, cx, cy)
        if cam.model == "PINHOLE":
            fx, fy, cx, cy = cam.params
            H, W = cam.height, cam.width

            server.scene.add_camera_frustum(
                f"/colmap/frame_{img_id}/frustum",
                fov=2 * np.arctan2(H / 2.0, fy),
                aspect=W / H,
                scale=0.15,
                image=image,
            )
        else:
            print(f"Skipping frustum for image {img_id}: camera model is {cam.model}")

    print("Open the viser URL shown above in your browser.")

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("root", help="Path to COLMAP root folder containing images/ and sparse/0/")
    args = parser.parse_args()

    main(args.root)
