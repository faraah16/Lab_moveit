#!/usr/bin/env python3
import cv2
import numpy as np

# Dictionnaire ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Taille du marker en pixels
marker_size = 300

# Générer 3 markers
markers = {
    0: "marker_red_table.png",
    1: "marker_blue_table.png",
    2: "marker_yellow_table.png"
}

output_dir = "/home/douaa/Lab_moveit/src/my_gazebo_world/markers/"

for marker_id, filename in markers.items():
    # Générer le marker (méthode corrigée)
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
    
    # Sauvegarder
    filepath = output_dir + filename
    cv2.imwrite(filepath, marker_image)
    print(f"Marker ID {marker_id} cree : {filename}")

print("\nTous les markers ArUco ont ete generes !")
