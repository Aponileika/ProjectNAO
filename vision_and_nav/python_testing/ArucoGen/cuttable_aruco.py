dpi = 300
mm_to_px = lambda mm: int(mm * dpi / 25.4)

a4_width_mm = 210
a4_height_mm = 297

a4_width_px = mm_to_px(a4_width_mm)
a4_height_px = mm_to_px(a4_height_mm)

num_markers_x = 3
num_markers_y = 4
#The dpi above is probably a bit wrong, this is ok.
square_size_mm = 45*1.04166666667 # each marker square is 40×40 mm
marker_size_mm = 40*1.04166666667 # marker itself is 30×30 mm inside the square
border_thickness_mm = 0.5*1.04166666667  # black border around each square

square_size_px = mm_to_px(square_size_mm)
marker_size_px = mm_to_px(marker_size_mm)
border_thickness_px = max(1, mm_to_px(border_thickness_mm))  # ensure at least 1 pixel
import cv2
import numpy as np

# ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

# Create A4 canvas
canvas = 255 * np.ones((a4_height_px, a4_width_px), dtype=np.uint8)

# Draw markers in grid
for y in range(num_markers_y):
    for x in range(num_markers_x):
        marker_id = y * num_markers_x + x
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

        # Compute top-left corner of this marker square
        start_x = x * square_size_px + (square_size_px - marker_size_px)//2
        start_y = y * square_size_px + (square_size_px - marker_size_px)//2

        # Place marker
        canvas[start_y:start_y+marker_size_px, start_x:start_x+marker_size_px] = marker_img

        # Draw black border for cutting
        top_left = (x * square_size_px, y * square_size_px)
        bottom_right = ((x+1) * square_size_px, (y+1) * square_size_px)
        cv2.rectangle(canvas, top_left, bottom_right, color=0, thickness=border_thickness_px)

# Save image
cv2.imwrite("./markers/A4_cuttable_aruco.png", canvas)
