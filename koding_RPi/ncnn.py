import time
import argparse
import requests
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2
import numpy as np
import ncnn

# Initialize the NCNN model
class NCNNModel:
    def __init__(self, param_file, bin_file):
        self.net = ncnn.Net()
        self.net.load_param(param_file)
        self.net.load_model(bin_file)

    def predict(self, image):
        mat = ncnn.Mat.from_pixels(image, ncnn.Mat.PixelType.PIXEL_BGR)
        extractor = self.net.create_extractor()
        extractor.input("data", mat)
        ret, mat_out = extractor.extract("output")
        return mat_out

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
    url = "http://yourloggingweb.com/log"  # Replace with your web logging URL
    response = requests.post(url, json=data)
    return response.status_code

# Define the detection and action function
def detect_and_act(frame, model, debug):
    results = model.predict(frame)

    # Assuming results is a list of detections with format [class_id, confidence, x1, y1, x2, y2]
    for result in results:
        class_id, confidence, x1, y1, x2, y2 = result
        if class_id == 0:  # Assuming '0' is the class ID for mice in your model
            data = {
                'timestamp': time.time(),
                'box': [x1, y1, x2, y2],  # Bounding box coordinates
                'confidence': confidence,  # Detection confidence
            }
            if debug:
                print(f"Detection: {data}")
                # Draw bounding box on the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"mouse {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                log_status = log_detection(data)
                print(f"Logged detection: {log_status}")

                # Activate the servo for water spray
                activate_servo()
    
    if debug:
        # Display the frame
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mouse detection with NCNN and PiCamera2")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to show frames and results in terminal")
    parser.add_argument("param_file", type=str, help="Path to the NCNN param file")
    parser.add_argument("bin_file", type=str, help="Path to the NCNN bin file")
    args = parser.parse_args()

    # Initialize the NCNN model
    model = NCNNModel(args.param_file, args.bin_file)

    try:
        while True:
            frame = camera.capture_array()
            if not detect_and_act(frame, model, args.debug):
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
