#!/usr/bin/env python

import cv2
from matplotlib import pyplot as plt

logo_cascade = cv2.CascadeClassifier('data/cascade.xml')

# opencv_createsamples -img landing_marker.jpg -bg bg.txt -info info/info.lst -maxxangle 0.1 -maxyangle 0.1 -maxzangle 1.6 -num 34320
# opencv_createsamples -info info/info.lst -num 34320 -w 24 -h 24 -vec positives.vec

img = cv2.imread('test_2.png')  # Source image
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img1 = cv2.imread('test_1.png')  # Source image
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
img2 = cv2.imread('test_3.png')  # Source image
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# image, reject levels level weights.
logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
logo1 = logo_cascade.detectMultiScale(gray1, scaleFactor=1.05)
logo2 = logo_cascade.detectMultiScale(gray2, scaleFactor=1.05)

for (x, y, w, h) in logo:
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)

for (x, y, w, h) in logo1:
    cv2.rectangle(img1, (x, y), (x + w, y + h), (255, 255, 0), 2)

for (x, y, w, h) in logo2:
    cv2.rectangle(img2, (x, y), (x + w, y + h), (255, 255, 0), 2)

plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.imshow(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
plt.imshow(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
plt.show()



