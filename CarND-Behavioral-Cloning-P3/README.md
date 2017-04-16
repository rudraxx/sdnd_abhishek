# **Behavioral Cloning**

Build a convolution neural network in Keras that predicts steering angles from images

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report

I have also added two sections at the bottom on:
- Lessons learnt
- What's next?

[//]: # (Image References)

[image1]: ./images/image1_visualize_data.png "VisualizeData"
[image2]: ./images/image2_cropped_images.png "CroppedImages"
[image3]: ./images/image3_histogram_dataset.png "Histogram"
[image4]: ./images/image4_translate_image.png "Image Translation"
[image5]: ./images/image5_brightness_augmentation.png "Brightness Augmentation"
[image6]: ./images/image6_horizontal_flipping.png "Horizontal Flip"
[image7]: ./images/image7_plot_losses.png "Plot Loss"
[image8]: ./images/image8_tensorboard_graph.png "Tensorboard Graph"

[//]: # (Video References)

[video1]: ./videos/gif_run1_sharp_turn_problems.gif "Sharp turn issues"
[video2]: ./videos/gif_run2.gif "Intermediate- issues on bridge "
[video3]: ./videos/gif_run3_final.gif "Great Demo run"


Before jumping into the details of the project, I'd like to show what the end result looks like.

The main objective was to train a deep neural network to learn from human driving behavior and then automatically steer the vehicle around the track based on the incoming camera image data.

<p align = 'center'>
  ![alt text][video3]
</p>

I have worked on dynamic system modeling in the past for coming up with physics based equations for various systems. In all those cases, we needed a good understanding of the system dynamics.  Being able to create a highly non linear response purely based on vision information completely blew me away.

After looking at the results, I wonder if we can add temporal logic to framework as well. If modifying images based on prior information can help neural nets to learn, won't the inherent knowledge of system dynamics help as well? But I digress..back to the original project.

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* **model.py**   - Contains the script to create and train the model
* **drive.py**   - Used for driving the car in autonomous mode
* **model.h5**   - Contains a trained convolution neural network
* **Readme.md**  I am using the README.md for the project as the project report and for summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing:

```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

- I have used 3 convolutional layers followed by 4 fully connected layers. (Lines 400-435 in model.py)

- Apart for this the model also uses cropping layer and lambda layer for feature normalization, and flatten layer.

- The model was created in Keras, and tested on AWS 	
g2.2xlarge EC2 instance.

I have used the Keras TensorBoard callback to save data for TensorBoard, and visualizing the architecture. But I wasn't able to get it to look as nice as when working in base Tensorflow:
![alt text][image8]

A few key points to note:

- All filters in the model are 3x3 sizes.

- Relu activation function has been used for all the layers except the last layer. The last layer should output just the logit (wx+b), since we are interested in getting the steering angle, not an activation.

- All the preprocessing like normalization and cropping is done in the keras model itself, so that we can directly input the images during test time. According to what I read, this also allows keras to process the computations in parallel, thus reducing the training time.


 It seems like for this case, the exact model structure was less of a factor a compared to having the right kind of training data.

#### 2. Attempts to reduce overfitting in the model

- The model contains dropout layers in order to reduce overfitting (model.py lines 426 and 429).

- I am using 50% dropout after each fully connected layer to reduce over fitting.

- For ensuring that the model doesn't see the validation data, I have used the train_test_split from the sklearn library.

- The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

- **Learning Rate:** The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 516).

- **Number of Epochs:**
  -  For the number of epochs, I had to look through the results, and ensure that the training and validation data were showing decreasing trend with each passing epoch.
  ![alt text][image7]

  - I have added early stopping in the model using Keras so that I don't have to manually do this later. But I wasn't able to utilize it well, because the loss doesn't seem to get worse for 2 consequent runs. I will have to play around with this to get intended behavior.

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road and lot of data augmentation.

I have used the grid lines to understand how much to crop the image by. The data looks something like this:
![alt text][image1]


For details about how I created the training data, see the next section.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

While I did experiment with different architectures, I have come to the conclusion  that so long as the model can extract hierarchical data,  that should be sufficient. Initially I was planning on using the NVidia model itself, but since I wanted to understand the relative importance of choosing the correct architecture vs training a give model appropriately, I decided to create one from scratch.

##### Case1:
- I started with a simple fully connected layer to validate that the workflow of training in AWS and running the simulator on my windows machine is working as expected. The performance was terrible but helped me in locking down the necessary commands.

##### Case 2:
- Once the above step was done, I basically created the 9 layer deep neural net with the necessary dropout layers as specified above, and started training it.

-  Trial and error was used to select number of epochs to prevent the validation data mean square error from increasing during the training. Because that would mean the model was starting to overfit the training dataset.

- The performance was much better while driving around track one. But still there were a few spots where the vehicle fell off the track. For example:

<p align='center'>
  ![car going off track][video1]
</p>
<p align=center>
  Vehicle going off track
</p>

I am using the following [online utility](http://image.online-convert.com/convert-to-gif) to convert videos to gif:



##### Case 3:

- To improve the driving behavior in these cases, I reran the simulator and recorded data for recovery action in those specific spots. This would enable the model to learn the correct behavior for those specific places.

- At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.
- Better performance, but if you notice, as soon as I reached the bridge, the car was vering to the left, and wasn't staying in the center.
<p align = 'center'>
  ![alt text][video2]
</p>
<p align=center>
  Slight left bias on the bridge
</p>

#### 2. Final Model Architecture

While the previous case corrected the behavior as soon as it reached the lane markings, I decided to add some more data for that region. And finally, this is what the performace looks like!

<p align = 'center'>
  ![alt text][video2]
</p>
<p align=center>
  Beautiful!
</p>


The final model architecture looks like this:

| Layer type           |  Output shape   | Parameters |
|:-------------:   | :-------------:|:-----------:|
|Input            |       (None, 320, 140, 3)| 0|
|cropping2d_1 (Cropping2D) |       (None, 90, 318, 3)  |  0    |      
|lambda_1 (Lambda)          |      (None, 90, 318, 3)  |  0   |       
|convolution2d_1 (Convolution2D) | (None, 88, 316, 16) |  448|
| maxpooling2d_1 (MaxPooling2D)   | (None, 44, 158, 16) |  0  |         
|convolution2d_2 (Convolution2D) | (None, 42, 156, 32) |  4640|        
|maxpooling2d_2 (MaxPooling2D)   | (None, 21, 78, 32)  |  0    |       
|convolution2d_3 (Convolution2D) | (None, 19, 76, 64)  |  18496 |      
|maxpooling2d_3 (MaxPooling2D)   | (None, 9, 38, 64)   |  0   |        
|flatten_1 (Flatten)             | (None, 21888)       |  0   |        
|dense_1 (Dense)                 | (None, 100)         |  2188900 |
|dropout_1 (Dropout)             | (None, 100)         |  0     |
|dense_2 (Dense)                 | (None, 50)          |  5050 |
|dropout_2 (Dropout)             | (None, 50)          |  0    |       
|dense_3 (Dense)                 | (None, 10)          |  510  |       
|dense_4 (Dense)                 | (None, 1)           |  11   |       

====================================================================================================
Total params: 2,218,055

Trainable params: 2,218,055

Non-trainable params: 0

=========================================================================

#### 3. Creation of the Training Set & Training Process

##### Creation of Training set:
- To capture good driving behavior, I first recorded two laps on track one using center lane driving. Here are some of the images. The title of the images shows the captured steering input:
![alt text][image2]

- The data was heavily biased towards straight line motion, and I knew right away that that would cause issues.
![alt text][image3]

- I have used only track 1 for the training as I want to see how well the model generalizes.

- So I trained for 2 more lap on the driving simulator, with focus on turning data, which led me to have lot more data, but the dataset size was getting very big causing slowdown, and eventually causing memory error on the AWS instance.

- This is where I decided to use the Keras generators, which enables creating new images based on existing images at runtime.

Some of the data proccessing techniques used in the model are:

1) **Cropping images:**
Cropping ensures that the learning can be focussed on the road edges, and not other things like hood of the car, surroundings etc.

![alttext][image2]


2) **Horizontal and vertical shifts:**
Enables adding data by moving the images left or right, and adding a small correction factor for the steering measurement

![alttext][image4]


3) **Brightness Augmentation:**
Enables learning in various lighting conditions

![alttext][image5]

4) **Horizontal flipping:**
Enables adding data with right hand turns instead of just left turns as seen on the track 1.
![alttext][image6]


- After the collection process, I had 8503 number of data points. But that wasn't enough to get the model to learn well enough to navigate turns.


**Observations:**
- One of the things that I realized that the keras version has a bug for the cropping layer. This prevents you from cropping only one axis. To overcome this, I had to crop 2 pixels off from the columns data as well.Keras

- Keras comes with a ImageGenerator, which would have been perfect to use. The problem was that I had to get random shift information in order to change the steering angle, and wasn't sure how to do that. This is why I created my own functions for the augmentation.

##### Training Process

The recorded image + steering angle data split into training and validation set. I have used a 20% split for the training data.
- Number of training samples : 6802
- Number of validation samples : 1701

- Based on this split data, 2 separate keras generators were used for training and validation datasets. (Line 387 in model.py)

## Lessons learnt
- Less data is not a hindrance, in case you can find ways to augment that data with data processing.

- Not using the Udacity dataset at all was initially frustrating. But the fact that I was able to get the model to learn purely based on my own data was a tremendous confidence booster. This also helped me in realizing how to collect relevant data. This will be valuable in future projects.

- Software can have bugs: Keras bug when cropping single axis of image.

## What's next
Lots of things...
- The model doesn't work on track 2. Need to understand if I need to train on that track, or can I do some more data augmentation that will help with driving on that track.

- Velocity information hasn't been taken into consideration. Steering should increase or decrease based on: velocity = cross(omega, turningRadius)

- No state dynamics have been taken into account. The network is looking at each instant, and taking action. Need to understand how the knowledge of vehicle dynamics can be included into this modeling paradigm.       
