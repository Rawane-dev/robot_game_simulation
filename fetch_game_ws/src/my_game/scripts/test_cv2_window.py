import cv2
import numpy as np

image = np.zeros((300, 400, 3), dtype=np.uint8)
cv2.putText(image, 'Test fenetre OpenCV', (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

cv2.imshow("Fenetre test", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

