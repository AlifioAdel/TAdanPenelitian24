# File: rat_detection.py

import cv2
import numpy as np
import time
from threading import Thread

# ----------- utils.py content -----------


class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=32):
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.stream.set(cv2.CAP_PROP_FPS, framerate)

        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

# ----------- yolo_manager.py content -----------


class YOLOManager:
    def __init__(self, config_path, weights_path, labels_path):
        self.net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
        self.labels = open(labels_path).read().strip().split("\n")
        self.layer_names = self.net.getLayerNames()
        self.output_layer_names = [self.layer_names[i[0] - 1]
                                   for i in self.net.getUnconnectedOutLayers()]

    def detect_objects(self, frame):
        (H, W) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        layer_outputs = self.net.forward(self.output_layer_names)
        end = time.time()

        boxes = []
        confidences = []
        class_ids = []

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)
        results = []

        if len(idxs) > 0:
            for i in idxs.flatten():
                x, y = boxes[i][0], boxes[i][1]
                w, h = boxes[i][2], boxes[i][3]
                confidence = confidences[i]
                class_id = class_ids[i]
                results.append(
                    (self.labels[class_id], confidence, (x, y, w, h)))

        return results

# ----------- main_picam.py content -----------


def main():
    config_path = "yolov3-tiny.cfg"
    weights_path = "yolov3-tiny.weights"
    labels_path = "coco.names"

    vs = PiVideoStream().start()
    time.sleep(2.0)

    yolo = YOLOManager(config_path, weights_path, labels_path)

    while True:
        frame = vs.read()
        results = yolo.detect_objects(frame)

        for label, confidence, box in results:
            if label == "rat":
                x, y, w, h = box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = f"{label}: {confidence:.2f}"
                cv2.putText(frame, text, (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    vs.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
