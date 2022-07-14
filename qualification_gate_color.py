#! /usr/bin/env python

import cv2
import numpy as np
from matplotlib import pyplot as plt

# reading image
frame = cv2.imread('Frame665.jpg')
cv2.imshow('image', frame)

b, g, r = cv2.split(frame)
# b=b-163 # ye yaha pe value change ki thi blue ki

# r=r+142
# clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
# equc_b= clahe.apply(b)
# equc_g = clahe.apply(g)
# equc_r = clahe.apply(r)
# equc = cv2.merge((equc_b, equc_g, equc_r))

def nothing(x):
	pass

cv2.namedWindow('track_', cv2.WINDOW_NORMAL)
# cv2.namedWindow('track_',cv2.WINDOW_NORMAL)

cv2.createTrackbar('bt','track_',0,255, nothing)
cv2.createTrackbar('gt','track_',0,255, nothing)
cv2.createTrackbar('rt','track_',0,255, nothing)

cv2.createTrackbar('lh','track_',0,180,nothing)
cv2.createTrackbar('uh','track_',0,180,nothing)
cv2.createTrackbar('ls','track_',0,255,nothing)
cv2.createTrackbar('us','track_',0,255,nothing)
cv2.createTrackbar('lv','track_',0,255,nothing)
cv2.createTrackbar('uv','track_',0,255,nothing)

while (1):

    frame = cv2.imread('Frame665.jpg')

    bt = cv2.getTrackbarPos('bt', 'track_')
    gt = cv2.getTrackbarPos('gt', 'track_')
    rt = cv2.getTrackbarPos('rt', 'track_')

    lh = cv2.getTrackbarPos('lh', 'track_')
    uh = cv2.getTrackbarPos('uh', 'track_')
    ls = cv2.getTrackbarPos('ls', 'track_')
    us = cv2.getTrackbarPos('us', 'track_')
    lv = cv2.getTrackbarPos('lv', 'track_')
    uv = cv2.getTrackbarPos('uv', 'track_')


    #for this pic
    # bt=24
    # gt=43
    # rt=222
    b_ = b - bt
    g_ = g - gt
    r_ = r + rt

    merg = cv2.merge((b_,g_,r_))
    cv2.imshow('after merge', merg)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    equc_b = clahe.apply(b_)
    equc_g = clahe.apply(g_)
    equc_r = clahe.apply(r_)
    # equc = cv2.merge((equc_b, equc_g, equc_r))
    equc = cv2.merge((equc_b, equc_g, r_))
    # frame=equc
    kernel = np.ones((3, 3), np.uint8)

    cv2.imshow('after clahe', frame)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #for blue
    # lower_blue = np.array([lh, ls, lv])

    # upper_blue = np.array([uh, us, uv])

    lower=np.array([lh,ls,lv])
    upper=np.array([uh,us,uv])

    mask = cv2.inRange(hsv, lower, upper)
    cv2.imshow('original mask', mask)
    mask = cv2.dilate(mask, kernel, iterations=6)
    cv2.imshow('dilated mask', mask)
    # mask = cv2.GaussianBlur(mask, (5, 5), 100)
    # cv2.imshow('blurred mask', mask)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
    
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        cnt = contours[max_index]
    
        cv2.drawContours(frame, [cnt], 0, (0,255,0), 3)
    
    # approx the contour a little
    # epsilon = 0.0005 * cv2.arcLength(cnt, True)
    # approx = cv2.approxPolyDP(cnt, epsilon, True)
    
    
    
    res = cv2.bitwise_and(frame, frame, mask=mask)
    
    # (x, y), radius = cv2.minEnclosingCircle(cnt)
    # center = (int(x), int(y))
    # radius = int(radius)
    # cv2.circle(frame, center, radius, (0, 255, 0), 2)

    # cv2.imshow('frame', frame)
    cv2.imshow('res', res)

    k = cv2.waitKey(5) & 0xFF

    if k == 27:
        break

cv2.destroyAllWindows()

# cap.release()
