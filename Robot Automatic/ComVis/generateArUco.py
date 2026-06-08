import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_6X6_250
)

marker = cv2.aruco.generateImageMarker(
    aruco_dict,
    30,      
    400      
)

cv2.imwrite("aruco30.png", marker)
