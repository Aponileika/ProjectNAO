import cv2
from ultralytics import YOLO

# -----------------------
# Load YOLO model
# -----------------------
model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    h, w = frame.shape[:2]

    # -----------------------
    # YOLO inference
    # -----------------------
    results = model(frame, verbose=False)[0]

    # -----------------------
    # define "danger zone" (front of robot)
    # -----------------------
    zx1 = int(w * 0.25)
    zy1 = int(h * 0.3)
    zx2 = int(w * 0.75)
    zy2 = int(h)

    # draw danger zone
    cv2.rectangle(frame, (zx1, zy1), (zx2, zy2), (255, 0, 0), 2)

    obstacle_detected = False

    # -----------------------
    # process detections
    # -----------------------
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        conf = float(box.conf[0])

        # filter weak detections
        if conf < 0.4:
            continue

        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # check if object overlaps danger zone
        overlap = not (x2 < zx1 or x1 > zx2 or y2 < zy1 or y1 > zy2)

        color = (0, 255, 0)

        if overlap:
            obstacle_detected = True
            color = (0, 0, 255)

        # draw box (NO class needed)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"object {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2)

    # -----------------------
    # status display
    # -----------------------
    if obstacle_detected:
        cv2.putText(frame, "OBSTACLE AHEAD!",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    3)
    else:
        cv2.putText(frame, "PATH CLEAR",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    3)

    # -----------------------
    # show output
    # -----------------------
    cv2.imshow("Generic Obstacle Detection", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()