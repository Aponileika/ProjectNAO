import cv2
import numpy as np
from webcamparam import K_JW, dist_coeffs  # your camera calibration

# Open camera
cam = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Board definition (4x4)
board_markersX = 4
board_markersY = 4
markerLength = 0.04
markerSeparation = 0.01
board = cv2.aruco.GridBoard((board_markersX, board_markersY), markerLength, markerSeparation, aruco_dict)
printxoffset = 0.022 + 0.014
printyoffset = 0.021 + 0.012

# Precompute board object points per marker
# board.getObjPoints() is a list of each marker's 4 corners in board coordinates
marker_obj_points_dict = {}
for idx, objPts in enumerate(board.getObjPoints()):
    marker_obj_points_dict[idx] = objPts.reshape(4, 3)  # each marker's 4 corners

axis_length = 0.05  # for drawing axes

while True:
    ret, image = cam.read()
    if not ret:
        break

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(image)

    if ids is not None and len(ids) > 0:
        # Flatten IDs
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # Collect all object points and image points for detected markers
        all_obj_points = []
        all_img_points = []
        for marker_corners, marker_id in zip(corners, ids):
            if marker_id >= len(board.getObjPoints()):
                continue  # skip if marker ID not on board
            objPts = marker_obj_points_dict[marker_id]
            all_obj_points.append(objPts)
            imgPts = marker_corners.reshape(4, 2).astype(np.float32)
            all_img_points.append(imgPts)

        if len(all_obj_points) > 0:
            all_obj_points = np.vstack(all_obj_points).astype(np.float32)
            all_img_points = np.vstack(all_img_points).astype(np.float32)

            # Solve PnP for the board
            success, rvec, tvec = cv2.solvePnP(
                all_obj_points,
                all_img_points,
                K_JW,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE
            )

            if success:
                cv2.drawFrameAxes(image, K_JW, dist_coeffs, rvec, tvec, axis_length)

                # Example: draw a grid on the board in X-Z plane
                N = 20
                mesh_points = []
                for i in range(N):
                    for j in range(N):
                        x = -j * markerLength - printxoffset + (N//2) * markerLength
                        y = -printyoffset
                        z = -i * markerLength
                        mesh_points.append([x, y, z])
                mesh_points = np.array(mesh_points, dtype=np.float32)
                imgpts, _ = cv2.projectPoints(mesh_points, rvec, tvec, K_JW, dist_coeffs)
                imgpts = imgpts.reshape(N, N, 2)

                for i in range(N):
                    for j in range(N):
                        pt = tuple(imgpts[i, j].astype(int))
                        if np.any(np.isnan(pt)) or np.any(np.isinf(pt)):
                            continue
                        cv2.circle(image, pt, 3, (0, 255, 0), -1)
                        if j < N - 1:
                            pt2 = tuple(imgpts[i, j + 1].astype(int))
                            if np.any(np.isnan(pt2)) or np.any(np.isinf(pt2)):
                                continue
                            cv2.line(image, pt, pt2, (0, 255, 0), 1)
                        if i < N - 1:
                            pt2 = tuple(imgpts[i + 1, j].astype(int))
                            if np.any(np.isnan(pt2)) or np.any(np.isinf(pt2)):
                                continue
                            cv2.line(image, pt, pt2, (0, 255, 0), 1)

    cv2.imshow("Board Detection", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
