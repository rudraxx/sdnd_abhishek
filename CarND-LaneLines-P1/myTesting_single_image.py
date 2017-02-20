# -*- coding: utf-8 -*-
"""
Created on Sun Feb 19 16:06:26 2017

@author: AbhishekBhat
"""

# -*- coding: utf-8 -*-
"""
Created on Sun Feb 19 14:39:04 2017

@author: AbhishekBhat
"""


import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import os
folder_name = "C:/Users/AbhishekBhat/sdnd_abhishek/CarND-LaneLines-P1/test_images"

##Step 1: Bring in 1 image. Then later replace this with a for loop for all images
#fnames = os.listdir(folder_name)
#
#for file in fnames:
#    current_file_path = os.path.join(folder_name,file)
#    if os.path.isdir(current_file_path):
#        print("Skipping " + file+ " . This is a directory")
#    else:
#        
#        fullname = folder_name+'/'+file
#        image = mpimg.imread(fullname)
#        #print ("Image name is image)
#        # Display this image
#        plt.imshow(image)
#        plt.show()
#    
    
fullname = folder_name+"\solidYellowCurve2.jpg"

image = mpimg.imread(fullname)
plt.imshow(image)
plt.show()


#%% Convert to grayscale
gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)

#    plt.imshow(gray,cmap = 'gray')
#    plt.show()

# smooth the image    
kernel_size = 5

blur_gray = cv2.GaussianBlur(gray,(kernel_size,kernel_size),0)
plt.imshow(blur_gray,cmap = 'gray')

# set up canny edge detector parameters
lower_threshold = 100
upper_threshold = 120

edges = cv2.Canny(blur_gray,lower_threshold,upper_threshold)
#    plt.imshow(edges,cmap = 'gray')
#    plt.show()

#%% Set up the focus area mask

mask = np.zeros_like(edges)
ignore_mask_color = 255

#Sepcify the vertices
imshape = image.shape
xsize = imshape[1]
ysize = imshape[0]

xmin = 20
xmax = xsize-20
ymin = math.floor(0.6*ysize)
ymax = ysize

lower_left = (xmin,ymax)
lower_right = (xmax,ymax)
top_right = (math.floor((xmin+xmax)/2) +20,ymin)
top_left  =  (math.floor((xmin+xmax)/2) - 20,ymin)

vertices = np.array([[lower_left,lower_right,top_right,top_left]],dtype=np.int32)

cv2.fillPoly(mask,vertices,ignore_mask_color) # create the polygon in mask

masked_image = np.bitwise_and(mask,edges)

#    plt.imshow(masked_image,cmap='gray')
#    plt.show()

#%% Set up the Hough Trasnform
rho = 1
theta = 1 * np.pi/180
threshold = 50#20
min_line_length = 10#20
max_line_gap = 5

line_image = np.copy(image)*0

lines = cv2.HoughLinesP(masked_image,rho,theta,threshold,min_line_length,max_line_gap)

#for line in lines:
#    for x1,y1,x2,y2 in line:
#        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),thickness=10)

slope_margin = 0.2
bias_margin = 50

average_slope_1 = 0.01
average_bias_1 = 0.01

average_slope_2 = 0.01
average_bias_2 = 0.01

idx=0
numLines_1 = 0
numLines_2 = 0

for line in lines:
    
    for x1,y1,x2,y2 in line:
        parameters = np.polyfit((x1,x2),(y1,y2),1)
        slope = parameters[0]
        bias  = parameters[1]

        # Initialize average to the slope and bias of the first line 
        if (idx==0):
            average_slope_1 = slope
            average_bias_1 = bias
            idx = idx+1
#            numLines_1 = 1

        if (~( (abs(slope-average_slope_1)<slope_margin) &
            (abs(bias-average_bias_1)<bias_margin)) & idx==1):
            
            average_slope_2 = slope
            average_bias_2 = bias
            idx = idx+1
            

        # For each iteration, check if slope and bias are close enough to the first line, 
        # else assign to second line
        
        if ( (abs(slope-average_slope_1)<slope_margin) &
            (abs(bias-average_bias_1)<bias_margin)):
#                numLines_1 = numLines_1+1
#                average_slope_1 = (average_slope_1+slope)/numLines_1
#                average_bias_1  = (average_bias_1+bias)/numLines_1
                average_slope_1 = (average_slope_1+slope)/2
                average_bias_1  = (average_bias_1+bias)/2
            
        elif( (abs(slope-average_slope_2)<slope_margin) & 
             (abs(bias-average_bias_2)<bias_margin)):
#                numLines_2 = numLines_2+1
#                average_slope_2 = (average_slope_2+ slope)/numLines_2
#                average_bias_2  = (average_bias_2 + bias)/numLines_2
                average_slope_2 = (average_slope_2+ slope)/2
                average_bias_2  = (average_bias_2 + bias)/2
            
xtop_1    = math.floor((ymin-average_bias_1)/average_slope_1)
xbottom_1 = math.floor((ymax-average_bias_1)/average_slope_1)

xtop_2    = math.floor((ymin-average_bias_2)/average_slope_2)
xbottom_2 = math.floor((ymax-average_bias_2)/average_slope_2)

cv2.line(line_image,(xtop_1,ymin),(xbottom_1,ymax),(255,0,0),thickness=2)
cv2.line(line_image,(xtop_2,ymin),(xbottom_2,ymax),(255,0,0),thickness=2)
        
#%% Plot the hough lines over the original image
copy_image = np.copy(image)
color_edges = np.dstack((edges,edges,edges))

combined_image = cv2.addWeighted(copy_image,1,line_image,1,0)
#combined_image = cv2.addWeighted(color_edges,0.8,line_image,1,0)
    
plt.imshow(combined_image)
plt.show()

## Now save this new image. Add _annotated so that you know which files are the new ones
#image_name = os.path.splitext(file)
#plt.imsave(fname=folder_name+'/abhishek/'+image_name[0]+'_annotated.jpg',
#           arr = combined_image)

