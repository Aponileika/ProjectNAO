import cv2
import numpy as np
from webcamparam import K_JW, dist_coeffs  # Your camera parameters

# Open camera
cam = cv2.VideoCapture(0)

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

marker_length = 0.048  # 48 mm in meters

# 3D points in marker frame
objPoints = np.array([
    [-marker_length/2,  marker_length/2, 0],  # top-left
    [ marker_length/2,  marker_length/2, 0],  # top-right
    [ marker_length/2, -marker_length/2, 0],  # bottom-right
    [-marker_length/2, -marker_length/2, 0]   # bottom-left
], dtype=np.float32)

#in world coordinates
N = 10
meshgridisempty = True
meshgrid = np.empty((N,N,3))
meshpoints = None
#target_id = 0
prev_rvec = None
prev_tvec = None
alpha = 0.8   

arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
arucoParams.cornerRefinementWinSize = 5
arucoParams.cornerRefinementMaxIterations = 30

while True:
    ret, image = cam.read()
    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(image)
    if ids is not None:
        ids = ids.flatten()
        for markerCorner, markerID in zip(corners, ids):
            #if(markerID != target_id):
            #    continue
            # Reshape corners
            pts = markerCorner.reshape((4,2))
            topLeft, topRight, bottomRight, bottomLeft = pts
            topLeft_int = tuple(map(int, topLeft))

            # Solve PnP
            imagePoints = markerCorner.reshape((4,2)).astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(objPoints, imagePoints, K_JW, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

            if success:
                # Draw marker axes
                cv2.drawFrameAxes(image, K_JW, dist_coeffs, rvec, tvec, 0.03)

                # Compute real-world width of top edge
                R, _ = cv2.Rodrigues(rvec)
                p1_marker = objPoints[0].reshape(3,1)  # top-left
                p2_marker = objPoints[1].reshape(3,1)  # top-right
                p3_marker = objPoints[2].reshape(3,1)  # bottom-right
                p4_marker = objPoints[3].reshape(3,1)  # bottom-left
                #print(f"tL, tR, bR, bL = {p1_marker}, {p2_marker}, {p3_marker}, {p4_marker}")
                offset = marker_length
                for i in range(N):
                    for j in range(N):
                        #assume flat surface
                        xval = p4_marker[0,0] - j*offset
                        yval = p4_marker[1,0]
                        #yval = p4_marker[1,0] - i*offset
                        zval = p4_marker[2,0] + i*offset
                        #zval = p4_marker[2,0] 

                        meshgrid[i,j,0] = xval
                        meshgrid[i,j,1] = yval
                        meshgrid[i,j,2] = zval
                meshpoints = meshgrid.reshape(-1, 3).astype(np.float32)
                meshgridisempty = False

                imgpts, _ = cv2.projectPoints(meshpoints, rvec, tvec, K_JW, dist_coeffs)

                imgpts = imgpts.reshape(N, N, 2)
                for i in range(N):
                    for j in range(N):
                        pt = tuple(imgpts[i,j].astype(int))
                        if np.any(np.isnan(pt)) or np.any(np.isinf(pt)):
                            continue
                        cv2.circle(image, pt, 3, (0,255,0), -1)
                
                        if j < N-1:
                            pt2 = tuple(imgpts[i,j+1].astype(int))
                            if np.any(np.isnan(pt2)) or np.any(np.isinf(pt2)):
                                continue
                            cv2.line(image, pt, pt2, (0,255,0), 1)
                
                        if i < N-1:
                            pt2 = tuple(imgpts[i+1,j].astype(int))
                            if np.any(np.isnan(pt2)) or np.any(np.isinf(pt2)):
                                continue
                            cv2.line(image, pt, pt2, (0,255,0), 1)

                p1_cam = R @ p1_marker + tvec
                p2_cam = R @ p2_marker + tvec

                width_m = np.linalg.norm(p2_cam - p1_cam)

                # Draw width on image
                cv2.putText(image,
                    f"Width: {width_m*1000:.1f} mm",
                    (topLeft_int[0], topLeft_int[1]-20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255,0,0),
                    2)

                # Optional: draw reprojected corners (magenta)
                projected_points, _ = cv2.projectPoints(objPoints, rvec, tvec, K_JW, dist_coeffs)
                projected_points = projected_points.reshape(-1,2)
                for p in projected_points:
                    cv2.circle(image, tuple(p.astype(int)), 5, (255,0,255), -1)

    cv2.imshow("Image", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
