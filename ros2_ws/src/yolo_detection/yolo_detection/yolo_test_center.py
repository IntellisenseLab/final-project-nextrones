#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO
import time

def main():
    # Load the YOLOv8n model
    print("Loading YOLOv8n model...")
    model = YOLO('yolov8n.pt')
    
    # Open the webcam (default is 0)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("YOLO Test Started. Press 'q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Run YOLOv8 inference on the frame
        results = model(frame, verbose=False)
        
        # Process results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get bounding box coordinates (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                cls = int(box.cls[0].cpu().numpy())
                label = model.names[cls]
                
                # Calculate center coordinates
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                if conf > 0.5:
                    # Draw bounding box
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    
                    # Draw center point
                    cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    
                    # Display label and center coords
                    text = f"{label} {conf:.2f} | Center: ({int(center_x)}, {int(center_y)})"
                    cv2.putText(frame, text, (int(x1), int(y1) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    print(f"Detected: {label} at center ({center_x:.1f}, {center_y:.1f})")

        # Display the resulting frame
        cv2.imshow('YOLOv8 Center Detection Test', frame)
        
        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
