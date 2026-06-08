import cv2
import cv2.aruco as aruco

cap = cv2.VideoCapture(0)

def findAruco(img, marker_size=6, total_markers=250, draw=True):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    key = getattr(
        aruco,
        f'DICT_{marker_size}X{marker_size}_{total_markers}'
    )

    arucoDict = aruco.getPredefinedDictionary(key)

    detector = aruco.ArucoDetector(arucoDict)

    bbox, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        print("Detected:", ids.flatten())

    if draw and ids is not None:
        aruco.drawDetectedMarkers(img, bbox, ids)

    return bbox, ids

while True:

    ret, img = cap.read()

    if not ret:
        print("Camera error")
        break

    bbox, ids = findAruco(img)

    cv2.imshow("Aruco Detection", img)

    key = cv2.waitKey(1)

    if key == ord('q'):
        break

print(img.shape)
print("IDs:", ids)

cap.release()
cv2.destroyAllWindows()
