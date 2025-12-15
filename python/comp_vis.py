# %%
import matplotlib.pyplot as plt
import numpy as np
import cv2
# from skimage.measure import label, regionprops
from scipy.ndimage import binary_fill_holes, binary_dilation

# %%
# img = cv2.imread('./data/karta-01.bmp')
# mask = np.all(img == [0, 0, 0], axis=-1)
# binary = mask.astype(np.uint8)
# filled = binary_fill_holes(binary).astype(np.uint8)
# structure = np.ones((5, 5), dtype=np.uint8)
# dilated = binary_dilation(filled, structure=structure, iterations=2).astype(np.uint8)
# plt.imshow(dilated, cmap='gray')
# plt.show()

img = plt.imread("./data/karta-01.bmp")
plt.imshow(img, cmap='gray')
plt.show()


# np.savetxt('../data/binary_map.txt', dilated, fmt='%d')
