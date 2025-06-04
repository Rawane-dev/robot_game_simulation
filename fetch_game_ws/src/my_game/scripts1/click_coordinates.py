#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import yaml
import os

coords = []
image_path = "/tmp/image_camera.png"
image = cv2.imread(image_path)
if image is None:
    print("❌ Image non trouvée à :", image_path)
    exit(1)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("✔️  Trou cliqué : ({}, {})".format(x, y))
        coords.append((x, y)) # Still append as tuple for internal use

window_name = "Cliquez sur les 42 trous (ligne 0 en haut)"
cv2.namedWindow(window_name)
cv2.setMouseCallback(window_name, mouse_callback)

while True:
    display_img = image.copy()
    for (x, y) in coords:
        cv2.circle(display_img, (x, y), 5, (0, 255, 0), -1)
    cv2.imshow(window_name, display_img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or len(coords) == 42:
        break

cv2.destroyAllWindows()

# ---
# Sauvegarde YAML
# ---
output_path = os.path.join(os.path.dirname(__file__), "grid_coords.yaml")

# Convert tuples to lists before dumping to YAML
coords_for_yaml = [list(coord) for coord in coords]

with open(output_path, 'w') as f:
    # Use default_flow_style=None to get block style for better readability
    yaml.dump(coords_for_yaml, f, default_flow_style=None)

print("✅ Coordonnées sauvegardées dans :", output_path)
