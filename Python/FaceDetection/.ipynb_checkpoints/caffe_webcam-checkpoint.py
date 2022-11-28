import cv2
import numpy as np

from os import path
import time

proto_path = path.join("res", "deploy.prototxt.txt")
model_path = path.join("res", "res10_300x300_ssd_iter_140000.caffemodel")

# Load model into DNN from OpenCV
net = cv2.dnn.readNetFromCaffe(proto_path, model_path)

# define a video capture object
vid = cv2.VideoCapture(0)
pTime = 0
print(vid.read()[1].shape)

(h, w) = vid.read()[1].shape[0], vid.read()[1].shape[1] 

# cv2.namedWindow('image')
# cv2.setMouseCallback('image', draw_circle)

confidence_threshold = 0.8

while(True):
    ret, frame = vid.read()
    
    frame = cv2.flip(frame, 1) 
    
    # Preprocess image resize 300x300, scale factor, image_size, mean values color channels
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
    # Send image to the neural network
    net.setInput(blob)
    detections = net.forward()
    
    # draw detections
    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > confidence_threshold:
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            text = f"{confidence * 100:.2f}"        
            y = startY - 10 if startY - 10 > 10 else startY + 10
            cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 0, 255), 2)
            cv2.putText(frame, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 255), 2)
            # print(startX, startY, endX, endY)
    
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(frame, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2)

    cv2.imshow('image', frame)
    
    # close ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break
        
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()