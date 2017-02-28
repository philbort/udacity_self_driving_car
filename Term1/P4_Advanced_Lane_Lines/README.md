##Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./example_images/undistorted1.png "Undistorted1"
[image2]: ./example_images/undistorted2.png "Undistorted2"
[image3]: ./example_images/undistorted3.png "Undistorted3"
[image4]: ./example_images/undistorted4.png "Undistorted4"
[image5]: ./example_images/undistorted5.png "Undistorted5"
[image6]: ./example_images/undistorted6.png "Undistorted6"
[image7]: ./example_images/undistorted7.png "Undistorted7"
[image8]: ./example_images/undistorted8.png "Undistorted8"
[image9]: ./test_images/test1.jpg "Test1"
[image10]: ./example_images/threshold1.png "Threshold1"
[image11]: ./example_images/transform1.png "Transform1"
[image12]: ./example_images/transform2.png "Transform2"
[image13]: ./example_images/linefit1.png "Linefit1"
[image14]: ./example_images/linefit2.png "Linefit2"
[image15]: ./example_images/final1.png "Final1"
[image16]: ./example_images/final2.png "Final2"
[image17]: ./example_images/final3.png "Final3"
[image18]: ./example_images/final4.png "Final4"
[image19]: ./example_images/final5.png "Final5"
[image20]: ./example_images/final6.png "Final6"
[video1]: ./videos/proc_project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

###Camera Calibration

####1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is defined in the `camera_calibration` function contained in the second code cell of the [IPython notebook](./P4.ipynb).  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection. I start with `9 corners` in `x-axis` and `6 corners` in `y-axis`. However, some of the calibration images only show a portion of the whole image and the we couldn't find `9` and `6` corners from them. As a result, I add a logic to search for fewer corners (e.g., `8 and 6`, `9 and 5`, etc.) once we couldn't detect corners from the original numbers. In this way, I'm able to find all the corners from the calibration images and able to use all of them for calibration.

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]
![alt text][image2]

The calibration result is saved as a `pickle` for future uses.

###Pipeline (single images)

####1. Provide an example of a distortion-corrected image.
The comparison between the test images and their undistorted images are shown below. Although it is pretty hard to tell the differences from naked eyes.

![alt text][image3]
![alt text][image4]
![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image9]

####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
I used a combination of color and gradient thresholds to generate a binary image (thresholding steps are under `4` in the [IPython notebook](./P4.ipynb).  The thresholds include:

 1. `Sobel` on `x` direction
 2. `Yellow` and `white` color detection from `RGB`
 3. `Yellow` and `white` color detection from `HSV`
 4. `Yellow` and `white` color detection from `HLS`
 5. `S channel` from `HLS`

A lot of credits are given to the community on `Slack` for the various ideas. The numbers for the thresholds are all hard-coded for now. A more robust approach will be explored later. Here's an example of my output for this step of the test image:

![alt text][image10]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is located under `3` in the [IPython notebook](./P4.ipynb). The `PerspectiveTransform()` function uses hard-coded pixel locations for both the original and new images and calculates the `M` and `Minv` matrices for perspective transform. Then, the `M` matrix is used by `cv2.warpPerspective` to convert the original image to its bird's view, while `Minv` is used to do transform in the opposite direction. Here's an example of my output for this step of the test image:

![alt text][image11]

And an example of a transformed image after thresholding is:

![alt text][image12]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The code for my line detection is located under `5` in the [IPython notebook](./P4.ipynb). I use a histogram to find the locations of the maximum pixels along the `y-axis` and assume those are the locations of the points on the line. A `RejectOutlier` function is used to compare the `x-axis` value of all the points to their median value and reject the ones who are far away from the median. Last, a 2nd order polynomial is used to fit the lane lines such as:

![alt text][image13]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The calculation of the radius of curvature of the line is obtained from the lecture. The vehicle position is determined by calculating the average `x coordinate` of the bottom left and bottom right points of the lines and comparing it with the middle point of the `x-axis`(i.e., 640). The deviation is then converted from pixels to meters. If the deviation is postive, vehicle is to the right of the center. If the deviation is negative, vehicle is to the left of the center. An example is shown as below:

![alt text][image14]

####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.
The implementation of the whole pipeline is located under `6` in the [IPython notebook](./P4.ipynb). Below are the final images for all six test images:

![alt text][image15]
![alt text][image16]
![alt text][image17]
![alt text][image18]
![alt text][image19]
![alt text][image20]

---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result][video1]

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

My main issue is that I haven't got enough time to work on this project. Work has been very busy at my company recently. Below are the improvements I will try to make once I have more time:
   
 1. Modularize the whole thing and convert the notebook to `py` files.
 2. Get rid of the hard-coded numbers as much as possible and make the code robust.
 3. Implement a class for the lines and use it to get temporal information from the videos to better smooth the lines.
 4. Tackle the challenge videos.
 5. Look back to Project 1 and try to find functions from there to help this project.
