import cv2
import torch
from ultralytics import YOLO

# Load your YOLOv8 model
try:
    #model = YOLO('/home/pi5/project/Mouse/runs/detect/train/weights/best.pt')
    model = YOLO('/home/pi5/project/Mouse/runs/detect/train4/weights/best_ncnn_model')
    #model = YOLO("/home/pi5/project/Mouse/runs/detect/train/weights/best.onnx")
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# Initialize Pi Camera
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Function to preprocess the frame before feeding it to the model
def preprocess_frame(frame):
    try:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))
        img = torch.from_numpy(img).float().div(255.0).permute(2, 0, 1).unsqueeze(0)
        return img
    except Exception as e:
        print(f"Error in preprocessing frame: {e}")
        return None
# Function to check for rats
def check_for_rats(results):
    rat_detected = False
    try:
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if model.names[box.cls] == "rat":  # Assuming "rat" is the class name for rats in your model
                    rat_detected = True
                    break
            if rat_detected:
                break
    except Exception as e:
        print(f"Error in checking for rats: {e}")
    return rat_detected

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Preprocess the frame
    img = preprocess_frame(frame)
    if img is None:
        continue

    # Perform detection
    try:
        results = model(img)
    except Exception as e:
        print(f"Error in model detection: {e}")
        break

    # Check for rats and print result
    if check_for_rats(results):
        print("Rat detected!")
    else:
        print("No rat detected.")

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
