# **Build a Traffic Sign Recognition Network**

Submission write up for the Project: CarND-LaneLines-P1  

---

## The goals / steps of this project are the following:


* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./ForReport/training_images.png "Training Images"
[image2]: ./ForReport/dataset_histogram.png "Histogram of Dataset"
[image3]: ./ForReport/augmented_dataset_histogram.png "Histogram of Augmented Dataset"
[image4]: ./ForReport/graph.png "Tensorboard graph"
[image5]: ./ForReport/model_architecture.png "CNN Architecture"
[image6]: ./ForReport/accuracy_plots.png "Accuracy Plot"

[image7]: ./additional_images/image1_60kmph.jpg "Traffic Sign 1"
[image8]: ./additional_images/image2_roundabout.jpg "Traffic Sign 2"
[image9]: ./additional_images/image3_doubleturn.jpg "Traffic Sign 3"
[image10]: ./additional_images/image4_dangerouscurveright.jpg "Traffic Sign 4"
[image11]: ./additional_images/image5_gostraightorright.jpg "Traffic Sign 5"
[image12]: ./ForReport/image_grayscaled.png "Grayscaled traffic signs"
[image13]: ./ForReport/hist1.png "Histogram for Traffic Sign 1 "
[image14]: ./ForReport/hist2.png "Histogram for Traffic Sign 2 "
[image15]: ./ForReport/hist3.png "Histogram for Traffic Sign 3 "
[image16]: ./ForReport/hist4.png "Histogram for Traffic Sign 4 "
[image17]: ./ForReport/hist5.png "Histogram for Traffic Sign 5 "
[image18]: ./ForReport/50kmph.png "50 kmph test image"
[image19]: ./ForReport/visualizelayer.png "Layer activation visualization"
[image20]: ./ForReport/newimages_label0.png "Augmented data for label 0"

## Rubric Points
 Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Submission Writeup / README

My code for the submission is on my [github repository](https://github.com/rudraxx/sdnd_abhishek/blob/master/CarND-Traffic-Sign-Classifier-Project/Traffic_Sign_Classifier.ipynb) as an IPython notebook.

### Data Set Summary & Exploration

#### 1. Basic summary of the data set and identifying where in my code was the summary done.

The code for this step is contained under the heading **Step 1:Data Set Summary & Exploration** in the second code cell of the IPython notebook.  

I am using Numpy for calculating the summary statistics of the traffic
signs data set:

* The size of training set is : 34799
* The size of validation set is : 4410
* The size of test set is   : 12630
* The shape of a traffic sign image is : (32, 32, 3)
* The number of unique classes/labels in the data set is : 43

#### 2.Exploratory visualization of the dataset

The code for this step is contained in the third code cell of the IPython notebook.  

Here is an exploratory visualization of the data set. I am displaying some of the test images, and then the histogram of the complete training dataset.

![alt text][image1]

![alt text][image2]


### Design and Test a Model Architecture

#### 1. Description of the preprocessing of the image data. What techniques were chosen and why did you choose these techniques?

The code for data preprocessing step is contained in the sixth code cell of the IPython notebook.

* The sixth code cell is where the actual data processing magic happens. I decided to convert the images to grayscale because the shapes of the traffic signs are more relevant than the color. So it makes sense to reduce the RGB image to grayscale.

* I also normalized the image data between 0.1 and 0.9. This normalized data  will help the gradient descent step move in a smoother fashion across the multiple input features.

#### 2. Describe how, and identify where in your code, you set up training, validation and testing data. How much data was in each set? Explain what techniques were used to split the data into these sets.

The code for data augmentation step is contained in the fourth and fifth code cell of the IPython notebook.

* The fourth code cell is the function - distort_data(). I created this function for distorting the training images, so that we can add/remove various methods of data augmentation easily, without affecting the main code.

* The fifth code cell is where I am actually doing the training dataset augmentation. This is to ensure that we have adequate data for all training classes. Based on the new histogram, we can see that we have enough training examples now.
![alt text][image3]  


The code for splitting the data into training and validation sets is contained in the first code cell of the IPython notebook.  Since the pickle files were already present, I used these as a good way of ensuring that the validation data and the test data are completely separate data sets. Since none of my data augmentation techniques were used with this data set, I thought using this data will be good way of checking how well my model generalizes.

My final training set had 67625 number of images as compared to the original 34799. My validation set and test set had 4410 and  12630 number of images respectively.


Here are some examples of original and augmented images. Notice how some of the images are rotated/skewed/tilted:

![!alt text][image20]

So what are the differences between the original data set and the augmented data set?
More importantly, why do we need that?

In cases where we don't have a lot of data, we can use the technique of data augmentation for adding to the data that we have. But we need to ensure that we aren't just copying the data over and over again. That won't help because we aren't adding any new information to the data pool. Care must be taken that the augmentation happens in a way that can be expected to happen in the real world, because that is what we are trying to recreate.. right?

We can expect to be looking at the traffic signs at an angle some of the times. In order to recreate that, some of the things that I did were:
* Rotating some of the images slightly.  
* Affine transform to skew the images.
* Perspective transform to give perception of depth wise rotation.

OpenCV has a some geometric transforms that help in doing this in an easy way. You can read about those [here](http://docs.opencv.org/trunk/da/d6e/tutorial_py_geometric_transformations.html).


#### 3. Describe, and identify where in your code, what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

The code for my final model is located in the seventh cell of the ipython notebook.

My final model consisted of the following layers:

| Layer         		|     Description	        					|
|:---------------------:|:---------------------------------------------:|
| Input         		  | 32x32x1 grayscale image   							|
| Convolution 3x3     | 1x1 stride, valid padding, outputs 30x30x32 	|
| RELU					      |												|
| Max pooling	      	| 2x2 stride,  outputs 15x15x32 				|
| Convolution 3x3     | 1x1 stride, valid padding, outputs 13x13x64 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 6x6x64 				|
| Fully connected		| outputs 512         									|
| Dropout       		| 50% dropout         									|
| Fully connected		| outputs 128         									|
| Dropout       		| 50% dropout         									|
| Fully connected		| outputs 43         									|
| Softmax				|     outputs 43						| |

I am using Tensorboard for visualizing the network. Using namespaces, I can specify how the nodes should show up, and is very easy to use! I would recommend the following video from the 2017 TensorFlow Dev summit 2017 to get a awesome introduction to Tensorboard. You can watch this video [here](https://www.youtube.com/watch?v=eBbEDRsCmv4).

Here is what the full graph for the network looks like:
![alt text][image4]

Followed by the deep network that I am using:
![alt text][image5]


#### 4. Describe how, and identify where in your code, you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

The code for training the model is located in the eighth cell of the IPython notebook.

At a high level, I wanted to run a parameter sweep to see how the changes in the hyper parameters would affect the final results. Tensorboard gives a way of visualizing multiple runs. I used that to see how the model is affected. While the looping worked really well, what I didn't anticipate was that with 50 epochs, and 20 total iterations, I ran out of the hard drive space on my AWS instance!
In hindsight, I should have initialized the setup with 32 or 64 GB, instead of the 16GB. But the again, hindsight has the benefit of 20/20 vision.

I did use dropout layers after both the fully connected layers so that I can ensure the model generalizes well.

To train the model, I used the ADAM optimizer. I ran the iteration for learning rates [1E-3, 1E-4], batch size: [100,128,200,256] and 50 epochs. While the training and validation accuracy was creeping up even after 50 epochs for the iterations with learning rate of 1E-4, I didn't have the time to spool up the system and rerun all tests with 100 epochs. That and $$$ of using AWS g2.2xlarge. :)

I had to finally settle on the learning rate of 1e-3, batch size of 128 and 20 epochs.

#### 5. Describe the approach taken for finding a solution. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

The code for calculating the accuracy of the model is located in the ninth cell of the Ipython notebook.


The final accuracy plots were again visualized with Tensorboard. This thing really is great at bringing data to life!
![alt text][image6]

* Training Accuracy    : 100%  
* Validation Accuracy  : 96.9 %

Finally I was read to see how well this model performs on the test data which had been kept under wraps till now. Like it was mentioned, it is imperative to use the test data only once, so that we get a true representation of how well this model will work in real world.

*Test Accuracy : 96.9%*  Woohooo!

* As described earlier, I iterated over learning rate and batch size
I initially started out with the LeNet architecture. But I didn't not include the drop out layers.
* Initially I didn't manage to get the validation accuracy to go over 90%. The main reason was that I had not augmented the dataset. With more augmented data, I set up 2 conv layers along with the drop out layers so that the model will generalize better.
* Like Vincent Vanhoucke had mentioned, I figured that a larger model a.k.a stretch pants approach is better than the skinny jeans just right mode size approach.

Some observations:
* We can expect traffic sign to come up anywhere in the image. Since convolution layers don't care about where in the image the object appears, this architecture is good choice for traffic sign detection.


### Test a Model on New Images

####1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image7]    ![alt text][image10]

![alt text][image11]  ![alt text][image9]

![alt text][image8]



#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. Identify where in your code predictions were made. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

The code for making predictions on my final model is located in the eleventh cell of the Ipython notebook.

  Here are the results of the prediction:

| Image			        |     Prediction	        					|
|:---------------------:|:---------------------------------------------:|
| 60 kmph       		|  20 kmph   									|
| Dangerous curve to the right     			  | Dangerous curve to the right
| Go straight or right 								|Go straight or right
| Double curve					    | Double curve											|
| Roundabout mandatory	      		| Priority road					 				|


The model was able to correctly guess 3 of the 5 traffic signs, which gives an accuracy of 60%. This doesn't compare favorably at all to the accuracy on the test set of 96%.

So I looked at the data that was being input to the model closely:
![!alt text][image12]

Nothing looks strange here. So I am not sure why the data is getting so confused with the first and last sign. I will have to plot out the images and visualize the activations to get some more insights.


#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction and identify where in your code softmax probabilities were outputted. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 12th cell of the Ipython notebook.


For the first image, the model seems to be really sure (75%) that this is a 20 kmph sign. The 60 kmph is third option and is down at 2 %.  Here are the plots of how the softmax output for the five images looks like.
![!alt text][image13] " Traffic Sign 1"

![!alt text][image14] " Traffic Sign 2"

![!alt text][image15] " Traffic Sign 3"

![!alt text][image16] " Traffic Sign 4"

![!alt text][image17] " Traffic Sign 5"

So I can see that the model is pretty sure of the 2,3,4th sign. For the 5th sign, the correct output is a close second.


#### Visualizing Layers of neural network:
Visualizing a specific layer can give us intuition on what that particular layer is capturing. This visualization seems to show that the circular pattern and the number 5,0 are being detected really well. The images look like an embossing on a metal plate. I think that means that the network is has trained to extract edges/ changes in contrast really well. Similar pattern emerges with other images as well.

![!alt text][image18] "Test Image"

![!alt text][image19] " Visualization of conv1 layer activation"

<!--

## Project: Build a Traffic Sign Recognition Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
In this project, you will use what you've learned about deep neural networks and convolutional neural networks to classify traffic signs. You will train and validate a model so it can classify traffic sign images using the [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). After the model is trained, you will then try out your model on images of German traffic signs that you find on the web.

We have included an Ipython notebook that contains further instructions
and starter code. Be sure to download the [Ipython notebook](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb).

We also want you to create a detailed writeup of the project. Check out the [writeup template](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/writeup_template.md) for this project and use it as a starting point for creating your own writeup. The writeup can be either a markdown file or a pdf document.

To meet specifications, the project will require submitting three files:
* the Ipython notebook with the code
* the code exported as an html file
* a writeup report either as a markdown or pdf file

Creating a Great Writeup
---
A great writeup should include the [rubric points](https://review.udacity.com/#!/rubrics/481/view) as well as your description of how you addressed each point.  You should include a detailed description of the code used in each step (with line-number references and code snippets where necessary), and links to other supporting documents or external references.  You should include images in your writeup to demonstrate how your code works with examples.  

All that said, please be concise!  We're not looking for you to write a book here, just a brief description of how you passed each rubric point, and references to the relevant code :).

You're not required to use markdown for your writeup.  If you use another method please just submit a pdf of your writeup.

The Project
---
The goals / steps of this project are the following:
* Load the data set
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report

### Dependencies
This lab requires:

* [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)

The lab enviroment can be created with CarND Term1 Starter Kit. Click [here](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) for the details.

### Dataset and Repository

1. Download the data set. The classroom has a link to the data set in the "Project Instructions" content. This is a pickled dataset in which we've already resized the images to 32x32. It contains a training, validation and test set.
2. Clone the project, which contains the Ipython notebook and the writeup template.
```sh
git clone https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project
cd CarND-Traffic-Sign-Classifier-Project
jupyter notebook Traffic_Sign_Classifier.ipynb
```

### Requirements for Submission
Follow the instructions in the `Traffic_Sign_Classifier.ipynb` notebook and write the project report using the writeup template as a guide, `writeup_template.md`. Submit the project code and writeup document. -->
