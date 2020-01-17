import numpy as np
import cv2
from matplotlib import pyplot as plt

rgb = cv2.imread("rgb0001.jpg")
thermal = cv2.imread("thermal0001.jpg")
combined = cv2.imread("thermal_to_rgb.jpg")

rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

plt.figure(1)
plt.subplot(2, 2, 1)
plt.title("RGB Image")
plt.imshow(rgb)
plt.subplot(2, 2, 2)
plt.title("Thermal Image")
plt.imshow(thermal)
plt.subplot(2, 2, 3)
plt.title("Align Thermal to RGB")
plt.imshow(combined)
color = cv2.applyColorMap(combined, cv2.COLORMAP_JET)
weighted = cv2.addWeighted(rgb, 1.0, color, 0.1, 0.3)
plt.subplot(2, 2, 4)
plt.title("Combined")
plt.imshow(weighted)

plt.show()
