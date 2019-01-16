
# **Deep Learning Traffic Sign Classifier**
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="img/sign.jpeg" width="480" alt="Combined Image" />

Overview
---
Traffic signs are an integral part of our transportation system. They provide critical information for drivers to promote their safety. Establishing a reliable traffic signs Classification mechanism is a significant step towards building semi-autonomous/autonomous vehicles. In this project,** Google's TensorFlow ** is used to implement a deep convolutional neural network to classify the [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). 

All the code for this project is contained in a [Jupyter notebook](./TrafficSignClassifier.ipynb). 

<br></br>
### Data Set Summary & Exploration



* ##### Training size = <font color="red" >34799 </font>
* ##### Test size =<font color="red" > 12630</font>
* ##### Validation size =<font color="red" > 4410</font>
* ##### Image data shape =<font color="red" > (32, 32)</font>
* ##### Number of Unique classes =<font color="red" > 43</font>

<br></br>

---
### Model Architecture




| Layer         		|     Description	        					|      OutPut     |
|:---------------------:|:---------------------------------------------:|:---------------:|
| Input         		| 32x32x1 Grayscale normalized image   			| NA              |
| Convolution 5x5     	| 1x1 stride, same padding  	                | 28x28x32        |
| RELU					|												|                 |
| Max pooling	      	| 2x2 stride  				                    | 16x16x32        |
| Convolution 5x5	    | 1x1 stride same padding 	                    | 10x10x64   |
| RELU					|												|  |
| Max pooling	      	| 2x2 stride       				                | 5x5x64|
| Flatten				| 5x5x64										| 1600               |
| Fully connected	1	| input: 1600        							|64|
| RELU					|												|                 |
| Fully connected	2	| input: 64        							|16|
| RELU					|												|                 |
| Fully connected	3	| input: 16        							|43|


<br></br>
---
#### Model Parameters

To train the model, I used the following parameters:

* rate = 0.001
* EPOCHS = 30
* BATCH_SIZE = 60
* weights_mean:  0.0
* weights_stddev:  0.1
* biases_mean:  0.0
* Type of optimizer: AdamOptimizer

More info:  [Traffic Sign Classification](./TrafficSignClassifier.ipynb). 

