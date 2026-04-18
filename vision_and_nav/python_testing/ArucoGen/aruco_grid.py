import cv2
import numpy as np
import os

# Make sure you have opencv-contrib-python installed
# pip install opencv-contrib-python

# Create dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

# Create the board
board = cv2.aruco.GridBoard(
    (4, 4),    # markersX, markersY
    0.04,      # marker length
    0.01,      # marker separation
    aruco_dict
)

# Desired output size in pixels
img_size = (500, 500)  # width, height

# This generateImage function *does exist* for boards
board_image = board.generateImage(img_size, marginSize=50, borderBits=1)

# Save
os.makedirs("./markers", exist_ok=True)
cv2.imwrite("./markers/aruco_board_generated.png", board_image)

# Optionally show
cv2.imshow("Board", board_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
