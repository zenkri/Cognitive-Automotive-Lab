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
from std_msgs.msg import Int8
from std_msgs.msg import String
import os
from multiprocessing import Process
import sys


rospy.init_node('dir', anonymous=True)
bridge = CvBridge()

testing = True
isDirSign = False

save_dir = os.path.dirname(os.path.realpath(__file__)) + '/images/color_img_new_'
suffix = '.png'

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 12, 3)
        self.conv2 = nn.Conv2d(12, 14, 3)
        self.conv3 = nn.Conv2d(14, 16, 3)
        self.conv4 = nn.Conv2d(16, 18, 3)
        self.conv5 = nn.Conv2d(18, 3, 1)
        self.pool = nn.MaxPool2d(2)
        self.fc1 = nn.Linear(3 * 4 * 4, 20)
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
        #print(x.shape)
        x = x.view(-1, 3 * 4 * 4)
        x = self.dropout(self.fc1(x))
        x = self.dropout(self.fc2(x))
        x = self.fc3(x)
        return x


class Net2(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 12, 3)
        self.conv2 = nn.Conv2d(12, 14, 3)
        self.conv3 = nn.Conv2d(14, 16, 3)
        self.conv4 = nn.Conv2d(16, 6, 1)
        #self.conv5 = nn.Conv2d(18, 6, 1)
        #self.conv6 = nn.Conv2d(48, 64, 3)
        #self.conv7 = nn.Conv2d(64, 128, 3)
        self.pool = nn.MaxPool2d(2)
        self.fc1 = nn.Linear(6 * 3 * 3, 40)
        self.fc2 = nn.Linear(40, 20)
        self.fc3 = nn.Linear(20, 2)
	self.dropout = nn.Dropout(p=0.2)

    def forward(self, x):
        #print(x.shape)
        #x = x.view(10, 1, 28, 28)
        x = F.relu(self.conv1(x))
        x = self.pool(x)
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = F.relu(self.conv3(x))
        #x = self.pool(x)
        x = F.relu(self.conv4(x))
        #x = self.pool(x)
        #x = F.relu(self.conv5(x))
        #x = F.relu(self.conv5(x))
        #x = F.relu(self.conv6(x))
        #x = F.relu(self.conv7(x))
        #x = self.pool(x)
        #x = self.pool(self.relu(self.conv5(x)))
        #print(x.shape)
        x = x.view(-1, 6 * 3 * 3)
        x = self.dropout(self.fc1(x))
        x = self.dropout(self.fc2(x))
        x = self.fc3(x)
        return x


directionSignsNet = torch.load(os.path.dirname(os.path.realpath(__file__)) + '/dirNet.pickle')
straightNet = torch.load(os.path.dirname(os.path.realpath(__file__)) + '/net2_dir_straight_cpu.pickle')
LRNet = torch.load(os.path.dirname(os.path.realpath(__file__)) + '/net2_dir_LR_cpu.pickle')

directionSignsNet.eval()
straightNet.eval()
LRNet.eval()

def get_filtered_channels(img):

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_blue = np.array([80, 80, 0])
    upper_blue = np.array([120, 255, 255])


    blue_mask = cv.inRange(hsv, lower_blue, upper_blue)

    return blue_mask



pub = rospy.Publisher('DirectionSign', Int8, queue_size=10)

height = 480
width = 640

dir_ma_filter = np.array([0, 0, 0, 0, 0, 0, 0])

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

new_u = 0
new_v = 0
new_w = 0
new_h = 0


def process_dirs(roi_color_img, roi_depth_img):
    global signFound
    global isDirSign
    global new_u, new_v, new_w, new_h

    #if signFound or isDirSign:
    #    roi_color_img = roi_color_img[new_v:new_v + new_h, new_u:new_u + new_w]
    #    roi_depth_img = roi_depth_img[new_v:new_v + new_h, new_u:new_u + new_w]
    #    signFound = False
    #else:
    #    new_u = 0
    #    new_v = 0


    B = get_filtered_channels(roi_color_img)


    kernel_erosion = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    kernel_dilation = cv.getStructuringElement(cv.MORPH_ELLIPSE, (15, 15))

    B = cv.morphologyEx(B, cv.MORPH_ERODE, kernel_erosion)
    B = cv.morphologyEx(B, cv.MORPH_DILATE, kernel_dilation)

    blue_contours = cv.findContours(B, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[-2:]

    signList = []

    if len(blue_contours[0]) > 0:
        for contour in blue_contours[0]:
            if cv.contourArea(contour) > 50:
                u, v, w, h = cv.boundingRect(contour)

                if w > h:
                    continue

                # increase the border of the bounding box by 10% in each direction
                margin = int(ceil(0.1 * h))
                u = max(0, u - margin)
                v = max(0, v - margin)
                w = min(639, u + w + margin) - u
                h = min(639, v + h + margin) - v

                distance = np.percentile(roi_depth_img[v:v+h, u:u + w], 50) / 1000.0

                pot_img = roi_color_img[v:v + h, u:u + w]

                if ((np.sum(pot_img) > 0) and (0.30 < distance < 2.60)):
                    if testing:
                        #cv.rectangle(roi_color_img, (u, v), (u + w, v + h), (255, 0, 0), 2)
                        pot_img = cv.cvtColor(cv.resize(pot_img, (28, 28)), cv.COLOR_BGR2GRAY)
                        pot_img = pot_img / 127.5 - 1.0
                        input_img = torch.Tensor(pot_img).view(1, 1, 28, 28)
                        _, isSign = torch.max(directionSignsNet(input_img), 1)

                        if isSign:
                            new_u, new_v, new_w, new_h = expand_ROI(u + new_u, v + new_v, w, h, 639, 279, 2)

                            _, straight = torch.max(straightNet(input_img), 1)
                            #cv.rectangle(roi_color_img, (u, v), (u + w, v + h), (0, 255, 255), 1)
                            #cv.putText(roi_color_img, 'STOP %.2f' % distance, (u, v + h + 10), cv.FONT_HERSHEY_PLAIN, h / 30.0, (0, 225, 225), 2)

                            if not straight:
                                _, right = torch.max(LRNet(input_img), 1)

                                if right:
                                        signList.append([1, distance, new_u, new_v, new_w, new_h])
                                else:
                                        signList.append([-1, distance, new_u, new_v, new_w, new_h])

                            else:
                                signList.append([0, distance, new_u, new_v, new_w, new_h])

                    else:
                        cv.imwrite(save_dir + str(time.time()) + '.png', cv.resize(pot_img, (28, 28)))

    global dir_ma_filter

    if len(signList) == 0:
        dir_ma_filter = shift(dir_ma_filter, 0)
        new_u = 0
        new_v = 0
    else:
        signList = np.array(signList)
        signList = signList[signList[:, 1].argsort()]
        dir_ma_filter = shift(dir_ma_filter, signList[0][0])
        new_u, new_v, new_w, new_h = (signList[0][2:]).astype(int)
        signFound = True


    (vals, counts) = np.unique(dir_ma_filter, return_counts = True)

    if np.max(counts) > 3:
        dir_idx = vals[np.argmax(counts)]
        pub.publish(dir_idx)
	if dir_idx != 0:
	   isDirSign = True
        else:
           isDirSign = False
    else:
        pub.publish(0)
        isDirSign = False


def process_img(color_msg, depth_msg, state):

    if state.data > 1:
        sys.exit()    

    color_img = cv.resize(bridge.imgmsg_to_cv2(color_msg, "bgr8"), (width, height))

    depth_img = cv.resize(bridge.imgmsg_to_cv2(depth_msg, "16UC1"), (width, height))

    roi_color_img = color_img[int(0.4167 * height):height - 1, :]

    roi_depth_img = depth_img[int(0.4167 * height):height - 1, :]

    process_dirs(roi_color_img, roi_depth_img)



if __name__ == '__main__':

    signFound = False

    sub_color = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image)
    sub_depth = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
    sub_state = message_filters.Subscriber("/current_task", Int8)

    timeSync = message_filters.ApproximateTimeSynchronizer([sub_color, sub_depth, sub_state], 10, 0.1, allow_headerless=True)
    timeSync.registerCallback(process_img)

    rospy.spin()

