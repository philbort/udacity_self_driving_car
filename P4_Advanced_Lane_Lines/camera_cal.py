import os
import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob

if __name__ == '__main__':

  # Parse the input argument
  parser = argparse.ArgumentParser(description='Camera calibration')
  parser.add_argument('-img_dir', type=str, default='camera_cal/', help='Directory of calibration images')
  args = parser.parse_args()


  objpoints = []
  imgpoints = []

  objp = np.zeros((6*9, 3), np.float32)
  objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

  directory = args.img_dir
  for file in os.listdir(directory):
    img = cv2.imread(os.path.join(directory, file))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
    if ret == True:
      imgpoints.append(corners)
      objpoints.append(objp)
      img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
      plt.imshow(img)
      plt.show()

  '''
  # Converting an image, imported by cv2 or the glob API, to grayscale:
  gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

  # Finding chessboard corners (for an 8x6 board):
  ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

  # Drawing detected corners on an image:
  img = cv2.drawChessboardCorners(img, (8,6), corners, ret)

  # Camera calibration, given object points, image points, and the shape of the grayscale image:
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

  # Undistorting a test image:
  dst = cv2.undistort(img, mtx, dist, None, mtx)

  '''