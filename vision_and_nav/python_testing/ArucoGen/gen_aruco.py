import cv2
import numpy as np

def generate_aruco_marker(dictionary_name="DICT_4X4_50", marker_id=0, marker_size=100, output_file="aruco_marker.png"):
    """
    Generate and save an ArUco marker image.

    :param dictionary_name: Name of predefined ArUco dictionary
    :param marker_id: ID of the marker (must be within dictionary range)
    :param marker_size: Size of marker image in pixels (square)
    :param output_file: Output filename
    """

    # Get the predefined dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))

    # Create blank image
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)

    # Generate marker
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

    # Save the marker
    cv2.imwrite(output_file, marker_image)

    print(f"ArUco marker saved as {output_file}")

if __name__ == "__main__":
    # Example usage
    generate_aruco_marker(
        dictionary_name="DICT_4X4_50",
        marker_id=0,
        marker_size=100,
        output_file="aruco_marker_0.png"
    )
