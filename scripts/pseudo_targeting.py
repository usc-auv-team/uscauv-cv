#!/usr/bin/env python
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial import distance as dist
import rospy
from std_msgs.msg import String

def orderPoints(pts):
    # use sys.maxsize for python 3
    lowest = [-1000,-1000]
    highest = [1000,1000]
    leftmost = [1000,1000]
    rightmost = [-1000, -1000]
    # print("\n")
    for i in pts:
    # print(i)
        if i[0] < leftmost[0]:
            leftmost = i
        if i[0] > rightmost[0]:
            rightmost = i
        if i[1] > lowest[1]:
            lowest = i
        if i[1] < highest[1]:
            highest = i
    # print("\n")
    # print("Lowest:")
    # print(lowest)
    # print("Highest")
    # print(highest)
    # print("Left:")
    # print(leftmost)
    # print("Right:")
    # print(rightmost)

    return np.array([lowest, highest, leftmost, rightmost], dtype="int32")
    # # sort the points based on their x-coordinates
    # xSorted = pts[np.argsort(pts[:, 0]), :]
    # # print("xSorted:")
    # # print(xSorted)
    # ind = np.argsort(pts, axis = 1)
    # ySorted = np.take_along_axis(pts, ind, axis = 1)
    # # print("ySorted:")
    # # print(ySorted)
    #
    # # grab the left-most and right-most points from the sorted
    # # x-roodinate points
    # leftMost = xSorted[:2, :]
    # rightMost = xSorted[2:, :]
    #
    # # now, sort the left-most coordinates according to their
    # # y-coordinates so we can grab the top-left and bottom-left
    # # points, respectively
    # leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    # (tl, bl) = leftMost
    #
    # # now that we have the top-left coordinate, use it as an
    # # anchor to calculate the Euclidean distance between the
    # # top-left and right-most points; by the Pythagorean
    # # theorem, the point with the largest distance will be
    # # our bottom-right point
    # D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    # (br, tr) = rightMost[np.argsort(D)[::-1], :]
    #
    # # return the coordinates in top-left, top-right,
    # # bottom-right, and bottom-left order
    # return np.array([tl, tr, br, bl], dtype="int32")

def checkOrientation(rect):
    angle = rect[2]
    (width, height) = rect[1]
    if width < height:
        angle = angle + 180
    else:
        angle = angle + 90
    if angle < 90:
        return "Right"
    elif angle > 90:
        return "Left"
    else:
        return "None"

def calcPoints(dir, box, frame):
    if dir == "None":
        return None
    else:
        ordered = orderPoints(box)
        (lowest, highest, leftmost, rightmost) = ordered

        start = [0,0]
        middle = [0,0]
        end = [0,0]
        if dir == "Right":
            # Start point is lowest point
            # Middle point is in between leftmost and topmost
            # End point is right most point
            start = lowest
            middle = [(leftmost[0] + highest[0])/2, (leftmost[1] + highest[1])/2]
            end = rightmost
        elif dir == "Left":
            # Start point is bottom most point
            # Middle point is in between rightmost and topmost
            # End point is leftmost point
            start = lowest
            middle = [(rightmost[0] + highest[0])/2, (rightmost[1] + highest[1])/2]
            end = leftmost
            # Order and draw points
        return np.array([start, middle, end], dtype="int32")

def extractImage(frame, rect):
    center, size, angle = rect[0], rect[1], rect[2]
    center, size = tuple(map(int, center)), tuple(map(int, size))

    height, width = frame.shape[0], frame.shape[1]
    print("Angle:")
    print(angle)
    if angle < -45:
        # angle +=90
        height, width = frame.shape[1], frame.shape[0]
        size = height, width
    # get rotation matrix
    M = cv.getRotationMatrix2D(center, angle, 1)
    # rotate image
    rotated = cv.warpAffine(frame, M, (width, height))
    # crop image
    cropped = cv.getRectSubPix(rotated, size, center)
    # cv.imshow('rotated', rotated)
    cv.imshow('cropped', cropped)

    crop_height, crop_width = cropped.shape[0], cropped.shape[1]
    q1 = cropped[0:crop_height/4, 0:crop_width/4]
    q2 = cropped[crop_height/2+crop_height/4:, 0:crop_width/4]
    q3 = cropped[0:crop_height/4, crop_width/2+crop_width/4:]
    q4 = cropped[crop_height/2+crop_height/4:, crop_width/2+crop_width/4:]
    cv.imshow('q1', q1)
    cv.imshow('q2', q2)
    cv.imshow('q3', q3)
    cv.imshow('q4', q4)
    return

def publishRectangle(box, pub):
    (lowest, highest, leftmost, rightmost) =  orderPoints(box)
    xmin = leftmost[0]
    xmax = rightmost[0]
    ymin = highest[1]
    ymax = lowest[1]

    print(xmin)
    print(xmax)
    print(ymin)
    print(ymax)
    print(" ")

    sent_str = "{" + "\"xmin\":" + str(xmin) + ", \"xmax\":" + str(xmax) + ", \"ymin\":" + str(ymin) + ", \"ymax\":" + str(ymax) + "}"
    pub.publish(sent_str);

def main():

    # Set up ROS Publisher
    refresh_rate = 50
    pub = rospy.Publisher('cv_detection', String, queue_size=10)
    rospy.init_node('target', anonymous = True)
    rate = rospy.Rate(refresh_rate)

    # cap = cv.VideoCapture(0)
    cap = cv.VideoCapture('test2.mp4')

    # frame_width = int(cap.get(3)) + 20
    # frame_height = int(cap.get(4)) + 20
    # out = cv.VideoWriter('path.avi',cv.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
    paused = False

    while(1):

        # frame = cv.imread('675.jpg')
        # frame = cv.imread('test.png')
        ret, frame = cap.read()
        # ret, frame = cap.read()
        if frame is None:
            cap = cv.VideoCapture('test2.mp4')
            ret, frame = cap.read()

        # Do edge detection to separate the edges from the picture
        edges = cv.Canny(frame, 0, 200)
        cv.imshow('edges', edges)

        # try blending the edges
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(9,9))
        dilated = cv.dilate(edges, kernel)
        # cv.imshow('dilated', dilated)

        # Try finding contours
        img2, contours, hierarchy = cv.findContours(dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # img2, contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # cv.drawContours(frame, contours, -1, (0, 255, 0), 3)

        # try drawing a rotated rectangle
        if len(contours) > 0:
            # find the biggest contours
            cnt = max(contours, key = cv.contourArea)
            rect = cv.minAreaRect(cnt)
            # orientation = checkOrientation(rect)
            # extractImage(frame, rect)
            # print(orientation)

        box = cv.boxPoints(rect)
        box = np.int0(box)
        publishRectangle(box, pub)

        # print("Box:")
        # print(box)

        # path_points = calcPoints(orientation, box, frame)
        #
        # if path_points is not None:
        #     colors = ((0, 0, 255), (240, 0, 159), (255, 0, 0), (255, 255, 0))
        #     for((x,y), color) in zip(path_points, colors):
        #         cv.circle(frame, (int(x), int(y)), 5, color, -1)
        cv.drawContours(frame, contours, -1, (0, 255, 0), 3)
        cv.drawContours(frame, [box], 0, (0,0,255), 2)
        # # cv.drawContours(frame_threshold, contours, -1, (0, 255, 0), 3)
        # # cv.drawContours(frame_threshold, [box], 0, (0,0,255), 2)
        # print(box)

        # cv.imshow('img', img)

        cv.imshow('frame', frame)
        # cv.imshow('frame_threshold', frame_threshold)
        # out.write(frame)

        if paused:
            i = cv.waitKey(0) & 0xFF
            if i == 32 and paused:
                paused = False
            elif i == 32 and not paused:
                paused == True
            elif i == 83:
                continue
            elif i == 27:
                break

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
        elif k == -1:
            continue
        elif k == 32:
            paused = True

    cv.destroyAllWindows()
    cap.release()
    # out.release()

    k = cv.waitKey(0) & 0xFF

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
