# **Finding Lane Lines on the Road**

Submission write up for the Project: CarND-LaneLines-P1  

---
## **Goals:**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on my work in a written report

## **Key Results:**
[//]: # (Image References)

[image0]: ./examples/abhishek/image_screenshot.jpg "Original"
[image1]: ./examples/abhishek/gray_screenshot.jpg "Grayscale"
[image2]: ./examples/abhishek/blur_screenshot.jpg "Smoothing"
[image3]: ./examples/abhishek/edges_screenshot.jpg "Canny Edge Detection"
[image4]: ./examples/abhishek/mask_screenshot.jpg "Region of interest"
[image5]: ./examples/abhishek/lines_screenshot.jpg "Hough Lines"
[image6]: ./examples/abhishek/combined_screenshot.jpg "Left and Right Lanes"

### Snapshots of the software pipeline for a single image:

![alt text][image0] "Original"

![alt text][image1] "Grayscale"

![alt text][image2] "Smoothing"

![alt text][image3] "Canny Edge Detection"

![alt text][image4] "Region of interest"

![alt text][image5] "Hough Lines"

![alt text][image6] "Left and Right Lanes"

---

## **Reflection:**

### My software pipeline consisted of the following 5 steps:
1. Converting the images to grayscale.
2. Smoothing the image using Gaussian blur.
3. Edge detection using Canny edge detection.
4. Specify the region of interest by masking the image.
5. Get left and right lane marking using Hough Transform.   


### Some insights about the Hough Transform:

- Tuning the Hough parameters is critical. Lower thresholds will lead to a lot of lines, while a high threshold will cause the pipeline to miss lane markings.
- Since we are interested in continuous lines instead of multiple dashed lines, we need to average out the output of Hough transform into 2 lines.
- In order to draw a single line on the left and right lanes, I modified the draw_lines() function by bucketing the output of Hough transform into two main buckets.
- I am using weighted average to get mean slope and mean bias for the two buckets.  

## **Potential shortcomings: **

1. The Hough Transform is trying to fit only straight lines. Need a different model to fit second or third order polyfit to the output of the canny edges.

2. The current implementation will only search for 2 lines. Left and right lane. What if there are 3 or 4 lanes? Need to account for that in the draw_lines() function.

3. Finally, the region of interest is static. I think this should change based on the scenario.

4. The threshold for the canny edge might have to be adaptive, in case we have foggy conditions. I think the upper and lower thresholds might need to be scaled down.  


## **Possible improvements to my pipeline: **

1. Adaptive thresholding for the Canny edge detector depending on the road condition.
2. Have 4-5 buckets for lanes, and then draw only specific ones depending on the number of lanes in the scene.
