from ultralytics import YOLO
import cv2

model = YOLO("YOLOv11s-Carmaker.pt")

image = cv2.imread("image.png")

results = model(image)


H=0.3
f=1000

detections = []

for i in range(len(results[0].boxes)) :
    box = results[0].boxes[i]
    x1, y1, x2, y2 = map(int , box.xyxy[0])
    h=abs(y1-y2)
    d=(H*f)/h
    detections.append(d)
    label=f" distance : {d:.2f}"
    cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
    cv2.putText(image,label,(x1,y1-10),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),2)

# for r in results:
#     boxes = r.boxes.xyxy

#     for box in boxes:
#         x1, y1, x2, y2 = map(int, box)

#         bbox_height = y2 - y1

#         distance = (H *f) / bbox_height

#         label = f"Dist: {distance:.2f}m"

#         cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
#         cv2.putText(image,label,(x1,y1-10),
#                     cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)


cv2.imwrite("output.jpg", image)
 
for i in range (len(detections)) :
    print(f"Cone {i+1}: distace = {detections[i]:.2f}\n")