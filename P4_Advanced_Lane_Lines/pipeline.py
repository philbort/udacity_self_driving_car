import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob

def camera_calibration(directory, nx, ny, draw = False):
  images = glob.glob(os.path.join(directory, 'calibration*'))
  objpoints = []
  imgpoints = []
  objp = np.zeros((nx*ny, 3), np.float32)
  objp[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

  for fname in images:
    # Read the calibration image
    img = cv2.imread(fname)
    # Converting an image, imported by cv2 or the glob API, to grayscale:
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Finding chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

    # If we detect the corners...
    if ret is True:
      imgpoints.append(corners)
      objpoints.append(objp)
      # If we want to draw the corners...
      if draw is True:
        # Drawing detected corners on the image
        img = cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
        plt.imshow(img)
        plt.show()
    else:
      print('Could not find corners from %s' %fname)

  # Camera calibration, given object points, image points, and the shape of the grayscale image:
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

  # Return the camera matrix and the distortion coefficients
  return mtx, dist

# Use calibrated camera parameters to undistort an image
def camera_undistort(img, mtx, dist):
  undist = cv2.undistort(img, mtx, dist, None, mtx)
  return undist

# Define a function that takes an image, gradient orientation,
# and threshold min / max values.
def abs_sobel_thresh(img, orient = 'x', thresh_min = 0, thresh_max = 255):
  # Convert to grayscale
  gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
  # Apply x or y gradient with the OpenCV Sobel() function
  # and take the absolute value
  if orient == 'x':
    abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
  elif orient == 'y':
    abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
  # Rescale back to 8 bit integer
  scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
  # Create a copy of the image
  binary_output = np.zeros_like(scaled_sobel)
  # Apply the threshold
  binary_output[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1

  return binary_output


# Define a function to return the magnitude of the gradient
# for a given sobel kernel size and threshold values
def mag_thresh(img, sobel_kernel = 3, mag_thresh = (0, 255)):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Take both Sobel x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize = sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize = sobel_kernel)
    # Calculate the gradient magnitude
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    # Rescale to 8 bit
    scale_factor = np.max(gradmag)/255 
    gradmag = (gradmag/scale_factor).astype(np.uint8) 
    # Create a binary image of ones where threshold is met, zeros otherwise
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

    # Return the binary image
    return binary_output
    
# Define a function to threshold an image for a given range and Sobel kernel
def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    # Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Calculate the x and y gradients
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the gradient direction, 
    # apply a threshold, and create a binary image result
    absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    binary_output =  np.zeros_like(absgraddir)
    binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1

    # Return the binary image
    return binary_output

if __name__ == '__main__':

  # Calibrate camera with calibration images
  mtx, dist = camera_calibration(directory = 'camera_cal/', nx = 9, ny = 6, draw = False)

  # Directory to save the processed images
  dest = 'proc_images'
  if not os.path.exists(dest):
    os.makedirs(dest)

  # Grab all the test images
  test_imgs = glob.glob(os.path.join('test_images/', 'test*.jpg'))
  for fname in test_imgs:
    # Get the original image
    img = cv2.cvtColor(cv2.imread(fname), cv2.COLOR_BGR2RGB)
    # Get the undistorted image
    undist = camera_undistort(img = img, mtx = mtx, dist = dist)
    # Plot the image
    fig = plt.figure()
    ax = plt.subplot(111)
    ax.imshow(undist)
    plt.xticks([])
    plt.yticks([])
    # Savefig doesn't work for jpg, use png instead
    savefig_name = 'undistorted_' + fname.split('/')[1].split('.')[0] + '.png'
    # Save the image
    fig.savefig(os.path.join(dest,savefig_name))

  undistorted_imgs = glob.glob(os.path.join(dest, '*.png'))
  for fname in undistorted_imgs:
    bianry_img = abs_sobel_thresh(cv2.imread(fname), 'x', 20, 100)
    fig = plt.figure()
    ax = plt.subplot(111)
    ax.imshow(bianry_img, cmap='gray')
    plt.show()