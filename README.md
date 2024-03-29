# Cognitive Automotive Lab. 2019
![BG](readme_images/BG.png)

This software is our proposed solution for the Cognitive Automotive Lab organized by the Institute of Measurement and Control Systems (MRT) at Karlsruhe Institute of Technology (KIT) 

## Structure
The software is composed of two main parts. A perception part, which aims to process RGBD images captured with a Microsoft Kinect 2 sensor. The second part of the code is: control. The subtasks of each part are illustrated in the following graphic. 

![Structure](readme_images/structure.png)

## Trafic signs detection
To increase the processing speed our group decided to not use the known detection algorithms like tiny YOLO ... but to create a task-specific solution, which can fulfill the requirements at high speed. The following graphic illustrates the detection of stop signs. Hier, a color-based region proposal in combination with a convolutional neural network (CNN) for the classification have been used.

![stop_sign_detection_structure](readme_images/perception_structure.png)

### Convolutional Neural Network
The following graphic shows the structure of the used CNN. The CNN is composed of 4 convolutional layers, 2 pooling layers, one bottleneck layer, and 3 fully connected layers. The total number of trainable parameters is 7557. The training is done with around a dozen thousand images.

![CNN](readme_images/CNN.png)

The following animated GIF shows the stop sign detection is action.

<p align="center">
  <img src="readme_images/stop_sign_detection.gif" width="800">
</p>

### Dynamic Region Of Interest
One key feature, which made such high frames per second (fps) rate possible is the implementation of what we call "dynamic" region of interest (ROI). The idea behind the dynamic ROI is very simple: Shrinking the ROI of the next frame to the region around the currently detected sign. Once no sign is detected, the ROI expands back to the original size. The dynamic ROI is demonstrated in the following GIF.

<p align="center">
  <img src="readme_images/dynamic_roi.gif", width="800">
</p>

### Arrow Traffic Sign
A part of the traffic sign detection task is the detection of arrow traffic signs. For this task 4 possible detection outputs are possible: 1)no sign 2)straight 3) right 4) left. For this task, a tree of 3 CNN networks as illustrated in the following graphic have been used. The main advantages of this structure are

* The ability to use the same data to train the tree small networks
* The fast processing of false proposed regions, as we don't need to go all the way through a complex network with four outputs.

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

The position of the obstacle is then evaluated to determine whether it interferes with the planned path. In the case of interference, the path is re-planed to avoid the obstacle. The following GIF demonstrates the algorithm.

<p align="center">
  <img src="readme_images/obstacle_avoidance.gif" width="800">
</p>

## Navigation in Unknown Environments
In regions where positioning is not possible, an autonomous driving vehicle should be able to safely navigate. This scenario is simulated through pillows, which the vehicle has to navigate through. This problem is solved in the following steps:

* Synchronize and resize depth and color images
* Distinguish pylon and ground in color image
* Match pixels between depth and color image (stereovision)
* Generate points cloud and transform it into the world coordinates
* Project points cloud to Occupancy Grid Map and uses Bayes rule to do the iteration
* Generate path incrementally dependent on endpoint direction of the current path
* Search a lateral range in the forward direction of endpoint
* Use potential field method to find the point with the lowest potential energy in the lateral range

<p align="center">
  <img src="readme_images/unknown_env.gif" width="800">
</p>

## Yielding for crossing pedestrians
One more challenge of the CAL was the detection of crossing pedestrian and yielding if he is crossing or driving if he is just side walking. This scenario was simulated with a remote-controlled robot. The processing steps are illustrated in the following graphic.

![ped_detection](readme_images/ped_detection.png)

<p float="left" allign="center">
  <img src="readme_images/ped1.gif" width="420" />
  <img src="readme_images/ped2.gif" width="420" /> 
</p>

## Steering Control
for the steering control, a serie of a Stanley controller followed by a PID controller have been used. The latter had a significant effect stabilizing the vehicle at high speeds and especially at curves. The steering control solution enabled a flawless running of the vehicle at maximal available speed 7,2km/h (4.47 miles/h). The two following GIFs show a comparison of the steering stability with  (left) and without (right) the PID controller enabled.


<p float="left" allign="center">
  <img src="readme_images/pid.gif" width="420" />
  <img src="readme_images/no_pid.gif" width="420" /> 
</p>

# Competition Day
We are very glad for being the winning team of the 2019 edition of the CAL competition.

<p align="center">
  <img src="readme_images/finals_vid.gif" width="800">
</p>

