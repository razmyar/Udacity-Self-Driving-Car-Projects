# Vehicle Detection and Tracking Pipeline

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)



<img src="imgs/pipelineFinal.png" width="480" alt="Combined Image" />
[YouTube Demo Link](https://youtu.be/DwudAsXfoIM)

Overview
---
This project represents a software pipeline to detect vehicles in a video.

**Pipeline Steps:**

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images.
* Train a Linear SVM classifier.
* Implement a sliding-window technique and use the trained classifier to search for vehicles in images.
* Create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.]
* Estimate a bounding box for the detected vehicles.]


All the code for this project is contained in this [Jupyter notebook](./Vehicle Detection.ipynb). 
