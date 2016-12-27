import os
import argparse
import cv2


if __name__ == '__main__':

  # Parse the input argument
  parser = argparse.ArgumentParser(description='Camera calibration')
  parser.add_argument('-img_dir', type='str', default='./camera_cal/', help='Directory of calibration images')
  args = parser.parse_args()

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