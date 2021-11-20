import cv2 as cv
import numpy as np

def readImage(imgData, camWidth, camHeight):
    img = np.fromstring( imgData, dtype='uint8').reshape((camHeight, camWidth, 4)) # covert image buffer to uint8 array
    #image = np.frombuffer(imgData, np.uint8).reshape((camHeight, camWidth, 4))

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
