import struct


def debug_images_binary(path):
    with open(path, "rb") as f:
        num_images = struct.unpack("<Q", f.read(8))[0]
        print(f"num_images = {num_images}")

        for i in range(num_images):
            print(f"\n--- image {i} at byte {f.tell()} ---")

            image_id = struct.unpack("<i", f.read(4))[0]
            qvec = struct.unpack("<dddd", f.read(32))
            tvec = struct.unpack("<ddd", f.read(24))
            camera_id = struct.unpack("<i", f.read(4))[0]

            name_bytes = bytearray()

            while True:
                c = f.read(1)

                if c == b"":
                    raise RuntimeError("EOF while reading image name")

                if c == b"\x00":
                    break

                name_bytes.extend(c)

            name = name_bytes.decode("utf-8")

            num_points2d = struct.unpack("<Q", f.read(8))[0]

            print(f"image_id = {image_id}")
            print(f"qvec = {qvec}")
            print(f"tvec = {tvec}")
            print(f"camera_id = {camera_id}")
            print(f"name = '{name}'")
            print(f"num_points2D = {num_points2d}")

            if num_points2d > 100000:
                print("!!! NumPoints2D is corrupted here")
                return

            f.seek(24 * num_points2d, 1)


debug_images_binary(
    "./sparse/snapshots/0/images.bin"
)
