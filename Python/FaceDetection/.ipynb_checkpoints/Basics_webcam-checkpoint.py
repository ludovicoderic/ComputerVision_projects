import cv2
import mediapipe as mp
import time

cap = cv2.VideoCapture(0)
pTime = 0

mpFaceDetection = mp.solutions.face_detection
mpDraw = mp.solutions.drawing_utils
confidence_threshold = 0.8
faceDetection = mpFaceDetection.FaceDetection(confidence_threshold)

while True:
    success, img = cap.read()

    imgRGB = cv2.flip(img, 1) #cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = faceDetection.process(imgRGB)
    # print(results)

    if results.detections:
        for id, detection in enumerate(results.detections):
            # mpDraw.draw_detection(img, detection)
            # print(id, detection)
            # print(detection.score)
            # print(detection.location_data.relative_bounding_box)
            bboxC = detection.location_data.relative_bounding_box
            ih, iw, ic = img.shape
            #print(img.shape)
            bbox = int(-bboxC.xmin * iw + 320 + (bboxC.width * iw)/2.5), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
            #print(bbox)
            cv2.rectangle(img, bbox, (0, 0, 255), 2)
            text = f"{detection.score[0] * 100:.2f}"        

            cv2.putText(img, text, (bbox[0], bbox[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2)
        cv2.imshow("Image", img)

                            # close ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break
            
# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()