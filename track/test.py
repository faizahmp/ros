import cv2
from ultralytics import YOLO
import numpy as np
import time

class WebcamObjectDetector:
    def __init__(self, model_path='yolov8n.pt'):
        """
        Initialize the webcam object detector with YOLOv8 model
        
        Args:
            model_path (str): Path to YOLOv8 model file (default: yolov8n.pt)
        """
        self.model = YOLO(model_path)
        self.cap = None
        
    def start_webcam(self, camera_index=0):
        """
        Start webcam capture
        
        Args:
            camera_index (int): Camera index (0 for default camera)
        """
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise ValueError(f"Could not open camera with index {camera_index}")
        
        # Set camera properties for better performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
    def detect_frame(self, frame, confidence_threshold=0.5):
        """
        Detect objects in a single frame
        
        Args:
            frame: Input frame from webcam
            confidence_threshold (float): Minimum confidence score for detections
            
        Returns:
            list: List of dictionaries containing detection information
        """
        # Run inference
        results = self.model(frame, verbose=False)
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get confidence score
                    confidence = float(box.conf[0])
                    
                    if confidence >= confidence_threshold:
                        # Get class ID and name
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]
                        
                        # Get bounding box coordinates (x1, y1, x2, y2)
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        
                        # Calculate center coordinates
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        
                        # Calculate width and height
                        width = x2 - x1
                        height = y2 - y1
                        
                        detection = {
                            'class_id': class_id,
                            'class_name': class_name,
                            'confidence': confidence,
                            'bounding_box': {
                                'x1': x1,
                                'y1': y1,
                                'x2': x2,
                                'y2': y2
                            },
                            'center': {
                                'x': center_x,
                                'y': center_y
                            },
                            'dimensions': {
                                'width': width,
                                'height': height
                            }
                        }
                        
                        detections.append(detection)
        
        return detections
    
    def draw_detections(self, frame, detections):
        """
        Draw bounding boxes and labels on frame
        
        Args:
            frame: Input frame
            detections: List of detection dictionaries
            
        Returns:
            annotated frame
        """
        annotated_frame = frame.copy()
        
        for detection in detections:
            bbox = detection['bounding_box']
            center = detection['center']
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, 
                         (int(bbox['x1']), int(bbox['y1'])), 
                         (int(bbox['x2']), int(bbox['y2'])), 
                         (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(annotated_frame, 
                      (int(center['x']), int(center['y'])), 
                      5, (0, 0, 255), -1)
            
            # Add label with background
            label = f"{detection['class_name']}: {detection['confidence']:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            
            # Draw label background
            cv2.rectangle(annotated_frame,
                         (int(bbox['x1']), int(bbox['y1'] - label_size[1] - 10)),
                         (int(bbox['x1'] + label_size[0]), int(bbox['y1'])),
                         (0, 255, 0), -1)
            
            # Draw label text
            cv2.putText(annotated_frame, label, 
                       (int(bbox['x1']), int(bbox['y1'] - 5)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # Add center coordinates text
            center_text = f"({int(center['x'])},{int(center['y'])})"
            cv2.putText(annotated_frame, center_text,
                       (int(center['x'] - 30), int(center['y'] - 10)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        return annotated_frame
    
    def run_detection(self, confidence_threshold=0.5, show_fps=True, print_detections=False):
        """
        Run real-time object detection on webcam feed
        
        Args:
            confidence_threshold (float): Minimum confidence score
            show_fps (bool): Display FPS on frame
            print_detections (bool): Print detection info to console
        """
        if self.cap is None:
            self.start_webcam()
        
        fps_counter = 0
        start_time = time.time()
        
        print("Starting webcam detection...")
        print("Press 'q' to quit, 'p' to toggle detection printing")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Detect objects
            detections = self.detect_frame(frame, confidence_threshold)
            
            # Draw detections
            annotated_frame = self.draw_detections(frame, detections)
            
            # Calculate and display FPS
            fps_counter += 1
            if show_fps:
                elapsed_time = time.time() - start_time
                if elapsed_time > 0:
                    fps = fps_counter / elapsed_time
                    cv2.putText(annotated_frame, f"FPS: {fps:.1f}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Display detection count
            cv2.putText(annotated_frame, f"Objects: {len(detections)}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Print detection info if enabled
            if print_detections and detections:
                print(f"\nFrame {fps_counter} - Found {len(detections)} objects:")
                for i, detection in enumerate(detections, 1):
                    print(f"  {i}. {detection['class_name']} "
                          f"(ID: {detection['class_id']}, "
                          f"Conf: {detection['confidence']:.3f}, "
                          f"Center: ({detection['center']['x']:.1f}, {detection['center']['y']:.1f}), "
                          f"Size: {detection['dimensions']['width']:.1f}x{detection['dimensions']['height']:.1f})")
            
            # Display frame
            cv2.imshow('YOLOv8 Webcam Detection', annotated_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                print_detections = not print_detections
                print(f"Detection printing: {'ON' if print_detections else 'OFF'}")
        
        self.cleanup()
    
    def get_single_detection(self, confidence_threshold=0.5):
        """
        Get detection results from a single frame
        
        Args:
            confidence_threshold (float): Minimum confidence score
            
        Returns:
            tuple: (detections_list, frame)
        """
        if self.cap is None:
            self.start_webcam()
        
        ret, frame = self.cap.read()
        if not ret:
            return [], None
        
        detections = self.detect_frame(frame, confidence_threshold)
        return detections, frame
    
    def cleanup(self):
        """
        Release resources
        """
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    # Initialize webcam detector
    detector = WebcamObjectDetector()
    
    try:
        # Method 1: Continuous detection with visualization
        print("Starting continuous detection...")
        detector.run_detection(
            confidence_threshold=0.5,
            show_fps=True,
            print_detections=True
        )
        
        # Method 2: Single frame detection (uncomment to use)
        # detections, frame = detector.get_single_detection()
        # if detections:
        #     print(f"Detected {len(detections)} objects:")
        #     for detection in detections:
        #         print(f"- {detection['class_name']}: {detection['confidence']:.3f}")
        
    except KeyboardInterrupt:
        print("\nStopping detection...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        detector.cleanup()

if __name__ == "__main__":
    main()