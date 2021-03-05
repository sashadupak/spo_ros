import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("no video")
    exit()

i = 33
while True:
    ret, frame = cap.read()
    if frame is None:
        break

    cv2.imshow("frame", frame)

    key = cv2.waitKey(33)
    if (key == ord("q")) or (key == 27):
        break
    elif key == 32: # space
        i += 1
        cv2.imwrite("../photos/face_" + str(i) + ".jpg", frame)

cap.release()
cv2.destroyAllWindows()
