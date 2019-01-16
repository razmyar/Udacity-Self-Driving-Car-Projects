# **Finding Lane Lines on the Road** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="examples/laneLines_thirdPass.jpg" width="480" alt="Combined Image" />

Overview
---

When we drive, we use our eyes to decide where to go.  The lines on the road that show us where the lanes are act as our constant reference for where to steer the vehicle.  Naturally, one of the first things we would like to do in developing a self-driving car is to automatically detect lane lines using an algorithm.

In this project, ** Python OpenCV **is used to identify lane lines on the road. The pipeline is developed on a series of individual images and later applied to a video stream. The applied OpenCV tools are color selection, a region of interest selection, grayscaling, Gaussian smoothing, Canny Edge Detection, and Hough Transform line detection. Here, the goal is to piece together a pipeline to detect the line segments in the image, then average/extrapolate them and draw them onto the image for display.


All the code for this project is contained in a [Jupyter notebook](./BasicLaneDetection.ipynb). 

