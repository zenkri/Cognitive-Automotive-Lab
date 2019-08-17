#!/usr/bin/env python

import torch
import torch.nn as nn
import torch.nn.functional as F
import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os
from pedestrian_detector.srv import *


bridge = CvBridge()

width = 28
height = 56


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 6, 3)
        self.conv2 = nn.Conv2d(6, 8, 3)
        self.conv3 = nn.Conv2d(8, 10, 3)
        self.conv4 = nn.Conv2d(10, 3, 1)
        # self.conv5 = nn.Conv2d(18, 6, 1)
        # self.conv6 = nn.Conv2d(48, 64, 3)
        # self.conv7 = nn.Conv2d(64, 128, 3)
        self.pool = nn.MaxPool2d(2)
        self.fc1 = nn.Linear(3 * 10 * 3, 40)
        self.fc2 = nn.Linear(40, 20)
        self.fc3 = nn.Linear(20, 2)
        self.dropout = nn.Dropout(p=0.2)

    def forward(self, x):
        # print(x.shape)
        # x = x.view(10, 1, 28, 28)
        x = F.relu(self.conv1(x))
        x = self.pool(x)
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = F.relu(self.conv3(x))
        # x = self.pool(x)
        x = F.relu(self.conv4(x))
        # x = self.pool(x)
        # x = F.relu(self.conv5(x))
        # x = F.relu(self.conv5(x))
        # x = F.relu(self.conv6(x))
        # x = F.relu(self.conv7(x))
        # x = self.pool(x)
        # x = self.pool(self.relu(self.conv5(x)))
        #print(x.shape)
        x = x.view(-1, 3 * 10 * 3)
        x = self.dropout(self.fc1(x))
        x = self.dropout(self.fc2(x))
        x = self.fc3(x)
        return x


robotNet = torch.load(os.path.dirname(os.path.realpath(__file__)) + '/net1_robot.pickle')
robotNet.eval()

def is_robot(msg):
    img = cv.cvtColor(cv.resize(bridge.imgmsg_to_cv2(msg.image, "bgr8"), (width, height)), cv.COLOR_BGR2GRAY)

    # normalize the gray values
    img = img / 127.5 - 1.0
    input_img = torch.Tensor(img).view(1, 1, 56, 28)
    _, isRobot = torch.max(robotNet(input_img.float()), 1)
    return_value = 0
    if isRobot:
	return_value = 1
    #return isRobot
    return pedestrian_classifierResponse(return_value)

def pedestrian_classifier_server():
    server = rospy.Service('pedestrian_classifier', pedestrian_classifier, is_robot)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('pedestrian_classifier_node')
    pedestrian_classifier_server()
