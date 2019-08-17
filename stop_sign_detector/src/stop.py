#!/usr/bin/python

import cv2 as cv
import numpy as np
import os.path
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
from math import ceil
import rospy
import message_filters
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import os
from multiprocessing import Pool
import sys
from std_msgs.msg import Int8

rospy.init_node('stop', anonymous=True)
bridge = CvBridge()

testing = True
isStop = False

last_stop_time = 0

save_dir = os.path.dirname(os.path.realpath(__file__)) + '/images/color_img_new_'
suffix = '.png'

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 6, 3)
        self.conv2 = nn.Conv2d(6, 8, 3)
        self.conv3 = nn.Conv2d(8, 10, 3)
        self.conv4 = nn.Conv2d(10, 12, 3)
        self.conv5 = nn.Conv2d(12, 3, 1)
        self.pool = nn.MaxPool2d(2)
        self.fc1 = nn.Linear(6 * 4 * 4, 20)
        self.fc2 = nn.Linear(20, 10)
        self.fc3 = nn.Linear(10, 2)
	self.dropout = nn.Dropout(p=0.2)

    def forward(self, x):
        #print(x.shape)
        #x = x.view(10, 1, 28, 28)
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = self.pool(x)
        x = F.relu(self.conv5(x))
        #print(x.shape)l
        x = x.view(-1, 3 * 4 * 4)
        x = self.dropout(self.fc1(x))
        x = self.dropout(self.fc2(x))
        x = self.fc3(x)
        return x

stopSignNet = torch.load(os.path.dirname(os.path.realpath(__file__)) + '/stopNet_cpu.pickle')
stopSignNet.eval()


def get_filtered_channels(img):

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_red = np.array([120, 80, 0])
    upper_red = np.array([180, 255, 255])

    #lower_blue = np.array([80, 80, 0])
    #upper_blue = np.array([120, 255, 255])

    red_mask = cv.inRange(hsv, lower_red, upper_red)
    #blue_mask = cv.inRange(hsv, lower_blue, upper_blue)

    return red_mask

stop_pub = rospy.Publisher('StopSignDetector', String, queue_size=10)

height = 480
width = 640

stop_ma_filter = np.array([0, 0, 0, 0, 0])

s_u = 0
s_v = 0
s_w = 0
s_h = 0

def shift(reg, val):
    reg = np.append(reg, val)
    reg = np.delete(reg, 0)
    return reg


def expand_ROI(roi_u, roi_v, roi_w, roi_h, img_w, img_h, factor):
    uu = int(max(0, roi_u - (factor - 1) * roi_w / 2))
    vv = int(max(0, roi_v - (factor - 1) * roi_h / 2))
    ww = int(min(uu + (factor + 1) * roi_w, img_w) - uu)
    hh = int(min(vv + factor * roi_h, img_h) - vv)

    return uu, vv, ww, hh


def process_img(color_msg, depth_msg, state):
	
    if state.data > 1:
    	sys.exit()


    color_img = cv.resize(bridge.imgmsg_to_cv2(color_msg, "bgr8"), (width, height))

    depth_img = cv.resize(bridge.imgmsg_to_cv2(depth_msg, "16UC1"), (width, height))

    roi_color_img = color_img[int(0.4167 * height):height - 1, :]

    roi_depth_img = depth_img[int(0.4167 * height):height - 1, :]

    global signFound
    global s_u, s_v, s_w, s_h

    #if signFound or isStop:
    #    roi_color_img = roi_color_img[s_v:s_v + s_h, s_u:s_u + s_w]
    #    roi_depth_img = roi_depth_img[s_v:s_v + s_h, s_u:s_u + s_w]
    #    signFound = False
    #else:
    #    s_u = 0
    #    s_v = 0


    R = get_filtered_channels(roi_color_img)


    kernel_erosion = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    kernel_dilation = cv.getStructuringElement(cv.MORPH_ELLIPSE, (15, 15))

    R = cv.morphologyEx(R, cv.MORPH_ERODE, kernel_erosion)
    R = cv.morphologyEx(R, cv.MORPH_DILATE, kernel_dilation)

    red_contours = cv.findContours(R, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[-2:]

    STOPList = []

    print((time.time() - last_stop_time))

    if (len(red_contours[0]) > 0) and ((time.time() - last_stop_time) > 5.0):
        for contour in red_contours[0]:
            if cv.contourArea(contour) > 50:
                u, v, w, h = cv.boundingRect(contour)

                if w > h:
                    continue

                margin = int(ceil(0.1 * h))
                u = max(0, u - margin)
                v = max(0, v - margin)
                w = min(639, u + w + margin) - u
                h = min(639, v + h + margin) - v

                distance = np.percentile(roi_depth_img[v:v + h, u:u + w], 50) / 1000.0

                pot_img = roi_color_img[v:v + h, u:u + w]

                if ((np.sum(pot_img) > 0) and (0.30 < distance < 2.60)):
                    if testing:
                        # cv.rectangle(roi_color_img, (u, v), (u + w, v + h), (255, 0, 0), 2)
                        pot_img = cv.cvtColor(cv.resize(pot_img, (28, 28)), cv.COLOR_BGR2GRAY)
                        pot_img = pot_img / 127.5 - 1.0
                        input_img = torch.Tensor(pot_img).view(1, 1, 28, 28)
                        _, isSTOP = torch.max(stopSignNet(input_img), 1)
                        if isSTOP:
                            s_u, s_v, s_w, s_h = expand_ROI(u + s_u, v + s_v, w, h, 639, 279, 2)
                            STOPList.append([1, distance, s_u, s_v, s_w, s_h])
		    else:
			cv.imwrite(save_dir + str(time.time()) + '.png', cv.resize(pot_img, (28, 28)))

    global stop_ma_filter
    global isStop

    if len(STOPList) == 0:
        stop_ma_filter = shift(stop_ma_filter, 0)
        s_u = 0
        s_v = 0
    else:
        STOPList = np.array(STOPList)
        STOPList = STOPList[STOPList[:, 1].argsort()]
        stop_ma_filter = shift(stop_ma_filter, 1)
        s_u, s_v, s_w, s_h = (STOPList[0][2:]).astype(int)
        signFound = True


    (vals, counts) = np.unique(stop_ma_filter, return_counts = True)

    stop_idx = vals[np.argmax(counts)]

    global last_stop_time

    if stop_idx == 1:
        stop_pub.publish('STOP')
        isStop = True
        last_stop_time = time.time()
    else:
        stop_pub.publish('DRIVE')
        isStop = False



if __name__ == '__main__':

    signFound = False

    sub_color = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image)
    sub_depth = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
    sub_state = message_filters.Subscriber("/current_task", Int8)


    timeSync = message_filters.ApproximateTimeSynchronizer([sub_color, sub_depth, sub_state], 10, 0.1, allow_headerless=True)
    timeSync.registerCallback(process_img)

    rospy.spin()

