# %%
import matplotlib.pyplot as plt
import numpy as np
# import cv2
# from skimage.measure import label, regionprops
from scipy.ndimage import binary_fill_holes

# %%
img = cv2.imread('./src/karta-01.bmp')
mask = np.all(img == [0, 0, 0], axis=-1)
binary = mask.astype(np.uint8)
filled = binary_fill_holes(binary).astype(np.uint8)
# img = plt.imread("./src/karta-01.bmp")

plt.imshow(filled, cmap='gray')
plt.show()

np.savetxt('./src/binary_map.txt', filled, fmt='%d')
