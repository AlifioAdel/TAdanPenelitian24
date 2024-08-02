import time
import argparse
import requests
import RPi.GPIO as GPIO
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize the YOLOv8 model (make sure to download or train a model specific for mice detection)
model = YOLO('yolov8-mice.pt')

# Setup GPIO for the servo (assuming Raspberry Pi setup)
SERVO_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM frequency
servo.start(0)

# Initialize PiCamera2
camera = Picamera2()
camera_config = camera.create_preview_configuration(main={"size": (640, 480)})
camera.configure(camera_config)
camera.start()

# Define the function to activate the servo
def activate_servo():
    servo.ChangeDutyCycle(7)  # Adjust this value to fit your servo's angle for spraying
    time.sleep(1)
    servo.ChangeDutyCycle(0)

# Define the function to send data to the web
def log_detection(data):
    #url = "http://yourloggingweb.com/log"  # Replace with your web logging URL
    response = requests.post(url, json=data)
    return response.status_code

# Define the detection and action function
def detect_and_act(frame, debug):
    results = model.predict(source=frame)

    for result in results:
        boxes = result.boxes
        for box in boxes:
            if box.cls == 'mouse':  # Assuming 'mouse' is the class name for mice in your model
                # Send detection result to the web logging system
                data = {
                    'timestamp': time.time(),
                    # 'box': box.xyxy.tolist(),  # Bounding box coordinates
                    'confidence': box.conf,  # Detection confidence
                }
                if debug:
                    print(f"Detection: {data}")
                    # Draw bounding box on the frame
                    x1, y1, x2, y2 = map(int, box.xyxy)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{box.cls} {box.conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    log_status = log_detection(data)
                    print(f"Logged detection: {log_status}")

                    # Activate the servo for water spray
                    #activate_servo()
    
    if debug:
        # Display the frame
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mouse detection with YOLOv8 and PiCamera2")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to show frames and results in terminal")
    args = parser.parse_args()

    try:
        while True:
            frame = camera.capture_array()
            if not detect_and_act(frame, args.debug):
                break
            time.sleep(1)  # Adjust the sleep time as necessary
    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        servo.stop()
        GPIO.cleanup()
        camera.close()
        if args.debug:
            cv2.destroyAllWindows()
