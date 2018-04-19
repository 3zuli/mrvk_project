import cv2
import sys
import numpy as np
from matplotlib import pyplot as plt

print(sys.argv)
if len(sys.argv)==2:
    fname = sys.argv[1]
else:
    fname = 'doge.jpg'
    # fname = 'under.jpg'
    # fname = 'over.jpg'

print("OpenCV Version : %s " % cv2.__version__)
image = cv2.imread(fname, 1)
# image = cv2.imread('doge.jpg', 1)
# image = cv2.imread('under.jpg', 1)

print(image.shape)
print(image.min(), image.max())
image_g = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print(image_g.shape)

hist = np.zeros(256)
histR = np.zeros(256)
histG = np.zeros(256)
histB = np.zeros(256)

for x in range(0, image_g.shape[0]):
    for y in range(0, image_g.shape[1]):
        hist[image_g.item(x, y)] += 1
        histB[image.item(x, y, 0)] += 1
        histG[image.item(x, y, 1)] += 1
        histR[image.item(x, y, 2)] += 1
hist_combined = histR + histG + histB

plt.figure(1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title("much vizs very wow")

plt.figure(2)
plt.subplot(221)
plt.hist(image_g.ravel(), 256, [0, 256])
plt.title("pyplot.hist()")

# print(hist)
# plt.figure(3)
# plt.plot(range(0,len(hist)), hist)
plt.subplot(223)
plt.plot(hist, 'k')
plt.title("gray")

plt.subplot(222)
plt.plot(hist_combined, 'k')
plt.title("R+G+B")

plt.subplot(224)
plt.plot(histR, 'r')
plt.plot(histG, 'g')
plt.plot(histB, 'b')
plt.title("RGB")

plt.show()