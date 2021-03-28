
# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.



# Task MP.1 --> If one wants to process a large image sequence with several thousand images and Lidar point clouds over night - in the current implementation this would push the memory of your computer to its limit and eventually slow down the entire program. So in order to prevent this, we only want to hold a certain number of images in memory so that when a new one arrives, the oldest one is deleted from one end of the vector and the new one is added to the other end. Therefore load data/image buffer is to be implemented

# Task MP.2 --> Implementation a selection of alternative detectors, which are HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT.

# Task MP.3 --> As we are focussing on a collision detection system , keypoints on the preceding vehicle are of special interest. Therefore, in order to enable a more targeted evaluation, we want to discard feature points that are not located on the preceding vehicle. Therefore  to remove all keypoints outside of a bounding box around the preceding vehicle. Box parameters used are : cx = 535, cy = 180, w = 180, h = 150. The coordinates are based on the Rectdatatype in OpenCV. 

# Task MP.4 --> Descriptor Extraction and Matching --> Implement a variety of keypoint descriptors to the already implemented BRISK method and make them selectable using the string 'descriptorType'. The methods integrated are BRIEF, ORB, FREAK, AKAZE and SIFT. The SURF is not a part of the mid-term project.

# Task Mp.5 --> task will focus on the matching part. The current implementation uses Brute Force matching combined with Nearest-Neighbor selection. FLANN is added as an alternative to brute-force as well as the K-Nearest-Neighbor approach. 

# Task MP.6 -->  implement the descriptor distance ratio test as a filtering method to remove bad keypoint matches.

# Task MP.7 -->  count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

# Task MP.8 -->  count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, use the BF approach with the descriptor distance ratio set to 0.8.

# Task MP.9 --> log the time it takes for keypoint detection and descriptor extraction. The results is entered into a spreadsheet and log files are generated and based on this information the suggestion of  the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles as follows.
 

All this above mid term tasks are implemented in the respective files and respective log files are generated which logs all the important information such as Key Points, Matched Key Points and Timing.

So based on the initial information, the best TOP3 Detector / Descritor combinations are as follows.

1) Detector/Descriptpr --> FAST + ORB |No .of  Keypoints --> 1670 | Matched Keypoints --> 1491 | Total Processing Time in ms --> 3.53e+15 ms |
2) Detector/Descriptpr --> FAST + BRISK | No .of  Keypoints --> 1491 | Matched Keypoints -->1348 | Total Processing Time in ms  --> 3.314e+15 ms |
3) Detector/Descriptpr --> SHITOMASI + ORB | No .of  Keypoints --> 1179 | Matched Keypoints --> 1067 | Total Processing Time in ms --> 2.22e+15 ms |


