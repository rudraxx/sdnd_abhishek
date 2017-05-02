# **Advanced Lane Finding**
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Create a software pipeline to identify the lane boundaries in a video.

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Create thresholded binary images using color transforms, gradients, etc.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

I have also added two sections at the bottom:
* Lessons Learnt
* What's next? How can we improve these results?

[//]: # (Image References)

[image1]: ./output_images/1_distortion_correction.png "Distortion Correction"
[image2]: ./output_images/2_vehicle_distortion_correction.png "Vehicle Image Distortion Correction"
[image3]: ./output_images/3_h_s_channel_binaries.png "H & S channel binaries"
[image4]: ./output_images/4_l_channel_gradient_thres.png "L channel Sobel X"
[image5]: ./output_images/5_combined_binaries.png "Combined binary image"
[image6]: ./output_images/6_colored_binary.png "Colored binary image"
[image7]: ./output_images/7_perspective_transform.png "Birds Eye View"
[image8]: ./output_images/8_histogram.png "Lane Histogram"
[image9]: ./output_images/9_bev_lane_markings.png "BEV Lane Markings"
[image10]: ./output_images/10_image_with_lane_masking.png "Final Image"
[image11]: ./examples/color_fit_lines.jpg "Example Polyfit"
[image12]: ./output_images/7_a_boundingbox_persp_transform.png "Bounding box"



[//]: # (Video References)
[video1]: ./videos/out_jittery_on_bridge.mp4 "No lane detection on bridge"
[video2]: ./videos/out_project_video.mp4 "Final Project video"

[video3]: ./videos/gif_jittery_on_bridge.gif "jitters on bridge"
[video4]: ./videos/gif_project_video.gif "Final Project gif"
[video5]: ./videos/gif_challenge_video.gif "Challenge gif"

[//]: # (Website References)
[website1]: (https://ezgif.com/)

Before jumping into the details, I'd like to show you how the end results look like.
![alt text][video4]

The full video is saved in the videos folder here:
[Link to the video](./videos/out_project_video.mp4)

The main objective was to ensure that we can detect the lane boundaries from the video input. The use of such a software pipeline can come in handy for open loop systems like Lane departure warning systems or maybe even drowsy driving detection.

The theory for this project was straight forward. But the implementation was more challenging because there is a lot of manual tuning that needs to be done.

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I have addressed each point in my implementation:

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  
I am using the README.md of the file itself as the project report. This way there is a single document for the reader to look into.

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the  code cells 1-2 of the IPython notebook located in "./code_submission.ipynb"


I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect **all** chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result in code cell #3 :

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

There are subtle modifications that we can see in the undistorted image. Notice how the white car on the right is pushed out to the right a little bit. Probably because of the flattening effect.

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

There are two sections for each of the filters that I have tested:
* Function prototyping  - Prototyping all functions
* End usage   - Using the prototyped functions in actual code.

This way, I can test the functionality in a separate section, and easily use the function at the end in the main pipeline. If something needs to change, I can do that in the function prototyping stage itself, and understand the effects.

I have used a combination of color and gradient thresholds to generate a binary image. Instead of directly working with the RGB image, I decided to convert the images to HLS colorspace since that seems to give better results for lane detection.

##### Color Thresholding
- I am thresholding on H and S channels. For each of these channels, I am also using clahe histogram thresholding so that the contrast improves. This might help with the gradient calculations.  

- Color thresholding is in the code section #5 of the iPython notebook. The output looks something like this:
![alt text][image3]

##### Gradient thresholding
- The L channel is used for gradient calculations in section #7.
![alt text][image4]

After doing all the thresholding, we decided to combine the three binaries so that we can use that for lane detection.
![alt text][image5]

Here is an image that shows the color binary. This image is composed of the following three separate binaries
 - R : H channel thresholding
 - G : S channel thresholding
 - B : Sobel X direction gradient.

 ![alt text][image6]



#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform in code cell # 11. I am using the ` cv2.warpPerspective` to get the transformation.  This takes an image as input as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
src = np.float32([[610,450],[720,450],[1150,720],[170,720]])  # orig
dst = np.float32([[640-350,0],[640+350,0],[640+350,720],[640-350,720]]) # topview       
M_perspective = cv2.getPerspectiveTransform(src,dst)
#Get the inverse perspective transform as well;
Minv_perspective = cv2.getPerspectiveTransform(dst,src)

```

Apart from calculating the transformation matrix for going from image to birds eye view, I am also calculating the inverse transformation matrix.

This resulted in the following source and destination points:

| Source        | Destination   |
|:-------------:|:-------------:|
| 610,450       | 290 ,0     |
| 720,450       | 990 ,0     |
| 1150,720      | 990 ,720   |
| 170,720       | 290 ,720   |

One thing that I had to keep in mind was that the src points need to be chose so that the lines are parallel. This is important for the perspective transform to do it's magic.
The bounding box looks something like this:
![alt text][image12]

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image7]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

- Lane lines were identified using a sliding window approach.   
- The first step was to identify the bottom of the lane by plotting the histogram of the lower half of the image:

  ![alt text][image8]

- This gets us the x coordinate of the left and right lane.  
- The code is in the code cell # 17 and is called `get_lane_arrays_with_sliding_windows()`

- Once the starting point has been detected, we can divide the image into specific number of windows and search the x,y coordinates of the non zero points within those windows. Thes points are marked in red and blue in the image below.

  ![alt text][image9]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

- I have instantiated the Left and Right lane using the Line class. This allows us to utilize same piece of code for the different lane types. The code for the class is in the iPython cell #15

- Based on the detected points, we then calculate the lane curvature using a 2nd order polynomial using the numpy `polyfit()` function. You can look at the class method `calculate_radius_curve() in` the Line class in code cell #15.

- For calculating the offset, I am calculating the center point of detected lanes, and comparing that to the center point of image. It is assumed that the x position of the image center aligns with the camera center/ vehicle center.  The function `calculate_center_offset_m()` is in the code cell #24

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

 - The plotting of the lane data back on the original image is done in function `draw_polygons()` , code cell #26 and `draw_text()` code cell #28.

  ![alt text][image10]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

The final video looks like this:

![alt text][video4]

The full video can be seen here:
[Link to my video result][video2]

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

***Problems faced:***

1) Throwing away certain fits.
- During the initial stages of the project, my pipeline was failing on the bridge. As soon as the vehicle reached the bridge, the lane detection was failing. This is how it looked like
![alt text][video3]

- Looking closer, I realized that I was using data from each frame, instead of checking the difference between the current fit and previous fit.

- For resolving this, I am checking the difference between the constant term of the fit. i.e coefficient fit[2]. Based on this, I throw away the new fit if that doesn't meet the requirements. (That means C coefficient in the image shown below.

![alt text][image11]

``` python            
  if  ( (left_lane.diffs[2]> diff_threshold) | (right_lane.diffs[2]>diff_threshold) ):
      ...
      ...

      if (left_lane.not_detected_count>5 or right_lane.not_detected_count>5):

  else:
      # reset the not detected count since we are in this loop.
      # Not detected count is used/updated when lanes not detected in consecutive images.
```


2) Deciding when to use sliding window, and when to use previous fit information

- I am keeping track of number of times that we lose track of the lanes. If we lose track of the lane for 5 consecutive frames, we go back to the sliding window for finding the fit.

-   Details are available in the code cell #32 / item 6:

3) Improvement for other driving conditions.

- I tried to test this on the other video, but that didn't see to work.
![alt text][video5]

- This looks really bad. I have spent some time trying out some additional filters, but clearly more works needs to be done.

#### Lessons Learnt

* Calibration of thresholds for filters is hard. Needs a lot of manual tuning.

* How do we decide which filters to use? Deciding which ones work in specific situations vs others is hard.  

#### What's next? How can we improve this results?

* See if we can fit a better model for the lines.
* Experiment with different filters.
