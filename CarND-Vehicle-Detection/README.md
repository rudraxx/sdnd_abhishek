# **Vehicle Detection and Tracking project**
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Create a software pipeline to detect vehicles in a video
---

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally apply a color transform and append binned color features, as well as histograms of color, to the HOG feature vector.
* Normalize the features and train a Linear Support Vector Classifier.
* Randomize the data and split into training and testing set.
* Implement a sliding-window technique and use the trained classifier to search for vehicles in images.
* Run the pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image0]: ./output_images/0_example_images.png
[image1]: ./output_images/1_color_hist_feature_vec.png
[image2]: ./output_images/2_spatial_vect.png
[image3]: ./output_images/3_hog_feature_vec.png
[image4]: ./output_images/4_svc_prediction_outputs.png
[image5]: ./output_images/5_sliding_windows.png
[image6]: ./output_images/6_hot_windows.png
[image7]: ./output_images/7_label_fcn_data.png

[//]: # (Video References)
[video1]: ./Videos/out_project_video_scalingerror.mp4
[video2]: ./Videos/out_project_video.mp4
[video3]: ./Videos/out_test_video.mp4

[video4]: ./Videos/gif_project_video_scalingerror.gif
[video5]: ./Videos/gif_project_video.gif
[video6]: ./Videos/gif_test_video.gif


[//]: # (Website References)
[website1]: (https://ezgif.com/)

Before jumping into the details, I'd like to show you how the end results look like.
![alt text][video5]

The full video is saved in the videos folder here:
[Link to the video](./Videos/out_project_video.mp4)


## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the code cell 12-14 of the IPython notebook.

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image0]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`).  I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like.

Here is an example using the `YCrCb` color space and HOG parameters of `orientations=9`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)`:


![alt text][image3]

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried a few different combinations of the HOG parameters. Main concern was to ensure that the feature vector doesn't increase to a high number. The HOG paper also mentions that they didnt see significant improvements for orientations>9, so that is what I finalized on.

 With the final config of parameters, my HOG feature vector is up at 5292 features.

#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

The code for the training SVM is in the iPython cells 20 and 21.

Firstly I normalize the feature vector in cell20, followed by the actual training in cell 21

Using a Linear SVC. The results looks something like this:
```
Using: 9 orientations 8 pixels per cell and 2 cells per block
Feature vector length: 5292
13.13 Seconds to train SVC...
Test Accuracy of SVC =  0.9879
```
98 % accuracy seems good enough for the initial testing. I plan on working on non linear classifiers to see if that improves the performance.

Here are some example images:
![alt text][image4]

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I initially started with creating multiple windows and running those over the image. But that caused the pipeline to run really slow, because we were extracting the HOG features multiple times.
Here is an example of that effort
![alt text][image5]

So instead I decided to use the find_cars code from the lesson. This is availabe in code cell 23

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on one scale using YCrCb 3-channel HOG features. I didnt end up using the spatial vectors, since I was getting worse results using the additional color and spatial features. Just goes on to prove that more fatures aren't always a good thing.

---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
The full video is in the Videos folder. [link to my video result](.Videos/project_video.mp4)

The result looks like this:
![alt text][video5]

One big problem that took me some time to figure out was that although I was getting good results in the training classifier (98% accuracy), but the results on the video were very jittery.
![alt text][video4]

The issue in the way matplotlib imports png files vs jpg files.
png files are imported with scaling of 0-1, while jpg are imported with range 0-255 range.
```
# Scale the imagae appropriately. VERY IMPORTANT. All the initial testing has been done on uint8 data.
       img2 = (img2 * 255).astype(np.uint8)
```
#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Apart from just detecting the vehicles, there are quite a few false positives.
In order to visualize how many times the vehicles are detected, we also have the heatmap set up to show us the most common areas of detection.
![alt text][ image6]

And then finally we use scipy labels `scipy.ndimage.measurements.label()` feature to get contiguous blocks to detect vehicles.

![alt text][image7]

Secondly, I am also combining the detections from multiple frames to make the detection more robust. I aggregate all the detection boxes in a global list, and then use the last 10 or 20 frames. The code for this is done in the line 14 onwards in the code cell 30.

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

1) One big problem that took me some time to figure out was that although I was getting good results in the training classifier (98% accuracy), but the results on the video were very jittery.
![alt text][video4]

2) Slow pipleline. Dont think I am running realtime right now. The video processing took 12 minutes, which is very slow. Need to figure out how to make this run faster. Would like to try out the cv2 hogdescriptor function, as I have heard that is faster.

3) Combining this with the lane detection. Will do that afterwards.
