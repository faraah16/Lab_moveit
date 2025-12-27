#!/usr/bin/env python3
import cv2

# Dictionnaire 6X6_250 (plus compatible)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

marker_size = 300

markers = {
    0: "marker_red_table.png",
    1: "marker_blue_table.png",
    2: "marker_yellow_table.png"
}

output_dir = "/home/douaa/Lab_moveit/src/my_gazebo_world/markers/"

for marker_id, filename in markers.items():
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
    filepath = output_dir + filename
    cv2.imwrite(filepath, marker_image)
    print(f"Marker ID {marker_id} cree avec DICT_6X6_250 : {filename}")

print("\nMarkers regeneres avec le dictionnaire 6X6_250 !")
