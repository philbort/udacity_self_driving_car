# Vehicle Detection
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/car_example.png
[image2]: ./output_images/not_car_example.png
[image3]: ./output_images/hog_car1.png
[image4]: ./output_images/hog_car2.png
[image5]: ./output_images/hog_car3.png
[image6]: ./output_images/hog_not_car1.png
[image7]: ./output_images/hog_not_car2.png
[image8]: ./output_images/hog_not_car3.png
[image9]: ./output_images/sliding_window_example.png
[image10]: ./output_images/detection_example1.png
[image11]: ./output_images/detection_example2.png
[image12]: ./output_images/detection_example3.png
[image13]: ./output_images/detection_example4.png
[image14]: ./output_images/detection_example5.png
[image15]: ./output_images/detection_example6.png
[video1]: ./project_video.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

###Histogram of Oriented Gradients (HOG)

####1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the first section `Explore the data` and second section `Feature selection` of the IPython notebook.  

I started by reading in all the `vehicle` and `non-vehicle` images.  Here are some example images of the `vehicle` and `non-vehicle` classes:

![alt text][image1] 
![alt_text][image2]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`).  I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like. I used `skimage.hog()` on `gray`, `hls`, and `hsv` channels. Below are some examples with HOG parameters of `orientations=9`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)`:


![alt text][image3] 
![alt text][image4]
![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]

####2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and realize the default values (see above) from the course are actually pretty good. Eventually I choose to use HOG with these parameters on the `hls` channel of the images.

####3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM using the HOG output on the `hls` channle of the images as well as the `color histogram` and the `bin spatials`. A `CalibratedClassifierCV` is used to wrap around the `LinearSVC` in order to get the predict probability, instead of prediction of `0` or `1`, to output. This can help to eliminate false positives in the pipeline. 20% of the training data is splitted to the test data, The linear SVM is able to achieve test accuracy of `0.9943`.

###Sliding Window Search

####1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I use three different sizes of windows to search `64 by 64`, `96 by 96`, and `128 by 128`. For the `y-axis`, the search is limited only from `400` to `600` so we don't have to search the sky or the nose of the car. For the `x-axis`, we only search from `600` to `1280` as we don't have cars on the left side. This will only work for the current video. If we are not in the left most lane, we have to modify the values to cover the potential cars on the left side as well.

Below is an example image of my sliding windows.

![alt text][image9] 

####2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to try to minimize false positives and reliably detect cars?

Ultimately I searched on two scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.  Here are some example images:

![alt text][image10]
![alt text][image11]
![alt text][image12]
![alt text][image13]
![alt text][image14]
![alt text][image15]



---

### Video Implementation

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video.mp4)


####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used blob detection in Sci-kit Image (Determinant of a Hessian [`skimage.feature.blob_doh()`](http://scikit-image.org/docs/dev/auto_examples/plot_blob.html) worked best for me) to identify individual blobs in the heatmap and then determined the extent of each blob using [`skimage.morphology.watershed()`](http://scikit-image.org/docs/dev/auto_examples/plot_watershed.html). I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap and bounding boxes overlaid on a frame of video:

![alt text][image5]

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

