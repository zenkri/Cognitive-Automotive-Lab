# Cognitive Automotive Lab. 2019
![BG](readme_images/BG.png)

This software is our proposed solution for the Cognitive Automotive Lab organized by the Institute of Measurement and Control Systems (MRT) at Karlsruhe Institute of Technology (KIT) 

## Structure
The software is composed of two main parts. A perception part, which aims to process RGBD images captured with a Microsoft Kinect 2 sensor. The second part of the the code is control. The subtasks of each part are illustrated in the following graphic. 

![Structure](readme_images/structure.png)

## Trafic signs detection
To encrease the processing speed our groupe decided to not use the known detection algorithms like tiny YOLO ... but to craete a task specific solution, which can fulfill the requirements athigh speed. The following graphic illustrates the detection of stop signs. Hier, a color based reagion proposal in combination with a convolutional neural network (CNN) for the classification have been used.

![stop_sign_detection_structure](readme_images/perception_structure.png)

### Convolutional Neural Network
The following graphic shows the structure of the used CNN. The CNN is composed of 4 convolutional layers, 2 pooling layers, one bottleneck layer and 3 fully connected layers. The total number of trainable parameters is 7557. The training is done with arround a dozen thousand images.

![CNN](readme_images/CNN.png)

The following animated GIF shows the stop sign detetction is action.

<p align="center">
  <img src="readme_images/stop_sign_detection.gif" width="800">
</p>

### Dynamic Region Of Interest
One key feature, wich made such high frames per second (fps) rate possible is the implementation of what we call "dynamic" region of intereset (ROI). The idea behind the dynamic ROI is very simple: Schring the ROI of the next frame to the region arround the currently detected sign. Once no sign is detected, the ROI expands back to the original size. The dynamic ROI is demonstrated in the following GIF.

<p align="center">
  <img src="readme_images/dynamic_roi.gif", width="800">
</p>

### Arrow Traffic Sign
A part of the traffic sign detection task is the detetction of arrow traffic signs. For this task 4 possible detection outputs are possible: 1)no sign 2)traight 3) right 4) left. For this task a tree of 3 CNN networks as illustrated in the following graphic have been used. The main advantages of this structure are

* The ability to use the same data to train the tree small networks
* The fast processing of false proposed reagions, as we don't need to go all the way through a complexer network with four aoutputs.

<p align="center">
  <img src="readme_images/dir_sign_tree.png" width="400">
</p>

## Obstacle Avoidance
The obstacle detection algorithm performs as follows:

* Apply ground removal using a contrast image
* Selecting ROI based on a depth range
* Use morphology to remove noise and smooth the contours
* Extracting bounding box of obstacles from ROI
* Evaluating the corresponding ROI in the color image to estimate if the detected object is an obstacle
* Returning the position of the obstacle

The position of the obstacle is then evaluated to determine weather it interfers with the planned path. In case of interference, the path is replaned to avoid the obstacle. The following GIF demostartes the algorithm.

<p align="center">
  <img src="readme_images/obstacle_avoidance.png" width="800">
</p>

