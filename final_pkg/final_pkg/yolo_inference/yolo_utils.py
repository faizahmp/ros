import cv2
from ultralytics import YOLO

model = YOLO('/home/ros2/ros2_lecture_ws/src/7_lectures/final_pkg/final_pkg/yolo_inference/yolov8n.pt')  # Use your model path if different


def run_yolo_on_image(frame, confidence_threshold=0.8):
    results = model(frame, verbose=False)
    detections = []

    for result in results:
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                confidence = float(box.conf[0])
                if confidence >= confidence_threshold:
                    class_id = int(box.cls[0])
                    class_name = model.names[class_id]
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    width = x2 - x1
                    height = y2 - y1

                    detections.append({
                        'class_id': class_id,
                        'class_name': class_name,
                        'confidence': confidence,
                        'bounding_box': {
                            'x1': x1, 'y1': y1,
                            'x2': x2, 'y2': y2
                        },
                        'center': {
                            'x': center_x, 'y': center_y
                        },
                        'dimensions': {
                            'width': width, 'height': height
                        }
                    })

    return detections
