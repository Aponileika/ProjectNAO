import cv2
import numpy as np
from webcamparam import K_JW, dist_coeffs  # your camera calibration

# ---------------------------
# Triangulation storage
# ---------------------------
clicked_points = []   # stores (C, ray) pairs
click_pixel = None

def triangulate_rays(C1, d1, C2, d2):
    A = np.stack([d1, -d2], axis=1)
    b = C2 - C1
    t = np.linalg.lstsq(A, b, rcond=None)[0]

    P1 = C1 + t[0] * d1
    P2 = C2 + t[1] * d2

    return (P1 + P2) / 2


def mouse_callback(event, x, y, flags, param):
    global click_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pixel = (x, y)


# ---------------------------
# Open camera
# ---------------------------
cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Camera failed to open")
    exit()

cv2.namedWindow("Board Detection")
cv2.setMouseCallback("Board Detection", mouse_callback)

# ---------------------------
# ArUco setup
# ---------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ---------------------------
# Board definition (4x4)
# ---------------------------
board_markersX = 4
board_markersY = 4
markerLength = 0.04
markerSeparation = 0.01

board = cv2.aruco.GridBoard(
    (board_markersX, board_markersY),
    markerLength,
    markerSeparation,
    aruco_dict
)

axis_length = 0.05

# ---------------------------
# Main loop
# ---------------------------
while True:
    ret, image = cam.read()
    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(image)

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # Estimate board pose (clean + correct)
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, board, K_JW, dist_coeffs, None, None
        )

        if retval > 0:
            cv2.drawFrameAxes(image, K_JW, dist_coeffs, rvec, tvec, axis_length)

            # ---------------------------
            # Handle click → ray
            # ---------------------------
            if click_pixel is not None:
                u, v = click_pixel

                # Pixel → normalized camera ray
                pt = np.array([[[u, v]]], dtype=np.float32)
                undistorted = cv2.undistortPoints(pt, K_JW, dist_coeffs)
                x, y = undistorted[0][0]
                ray_cam = np.array([x, y, 1.0])

                # Camera pose → world
                R, _ = cv2.Rodrigues(rvec)

                # Camera center in world coordinates
                C = -R.T @ tvec
                C = C.reshape(3)

                # Ray direction in world
                ray_world = R.T @ ray_cam
                ray_world /= np.linalg.norm(ray_world)

                clicked_points.append((C, ray_world))
                print(f"Stored ray #{len(clicked_points)}")

                click_pixel = None  # reset click

            # ---------------------------
            # Triangulate when 2 rays exist
            # ---------------------------
            if len(clicked_points) >= 2:
                C1, d1 = clicked_points[-2]
                C2, d2 = clicked_points[-1]

                P = triangulate_rays(C1, d1, C2, d2)

                print("Triangulated 3D point (board frame):", P)

                # Optional: project back to image for visualization
                obj_pt = np.array([[P]], dtype=np.float32)
                imgpt, _ = cv2.projectPoints(obj_pt, rvec, tvec, K_JW, dist_coeffs)

                x_img, y_img = imgpt[0][0].astype(int)
                cv2.circle(image, (x_img, y_img), 6, (0, 0, 255), -1)

    cv2.imshow("Board Detection", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()