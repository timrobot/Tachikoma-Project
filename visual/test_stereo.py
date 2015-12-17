import numpy as np
import cv2
import sys
import customdraw.draw as cdm

nmatches = 35

# get the image names
image1name = "leftimg00.png"
image2name = "rightimg00.png"

# read in the images
img1 = cv2.imread(image1name, 0)
img2 = cv2.imread(image2name, 0)

# start ORB detector
orb = cv2.ORB()

# find the keypoints with ORB
kp1 = orb.detect(img1, None)
kp2 = orb.detect(img2, None)

# compute the descriptors with ORB
kp1, des1 = orb.compute(img1, kp1)
kp2, des2 = orb.compute(img2, kp2)

# draw keypoint location
image1 = cv2.drawKeypoints(img1,kp1,color=(0,255,0), flags=0)
image2 = cv2.drawKeypoints(img2,kp2,color=(0,255,0), flags=0)

# now try to match the points
bf = cv2.BFMatcher() # create matcher
matches = bf.match(des1,des2)
matches = sorted(matches, key=lambda x:x.distance)[:nmatches]
image3 = cdm.drawMatches(img1,kp1,img2,kp2,matches)
cv2.imshow("ORB", image3)
cv2.waitKey(0)
cv2.imwrite("2_ORB.png", image3)
pts1 = []
pts2 = []
for i in range(len(matches)):
  pts2.append(kp2[matches[i].trainIdx].pt)
  pts1.append(kp1[matches[i].queryIdx].pt)

pts1 = np.float32(pts1)
pts2 = np.float32(pts2)

# now compute the fundamental matrix
F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_RANSAC)

# Find epilines corresponding to points in right image (second image) and
# drawing its lines on left image
lines1 = cv2.computeCorrespondEpilines(pts2.reshape(-1,1,2), 2,F)
lines1 = lines1.reshape(-1,3)
img5,img6 = cdm.drawLines(img1,img2,lines1,pts1,pts2)

# Find epilines corresponding to points in left image (first image) and
# drawing its lines on right image
lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,F)
lines2 = lines2.reshape(-1,3)
img3,img4 = cdm.drawLines(img2,img1,lines2,pts2,pts1)

cv2.imshow("left epipole", img5)
cv2.imshow("right epipole", img3)
cv2.waitKey(0)
cv2.imwrite("3_LEFT.png", img5)
cv2.imwrite("3_RIGHT.png", img3)
print "Fundamental:"
print F
print ""

# grab the stereo images
stereo = cv2.StereoBM()
disparity = stereo.compute(img1,img2, disptype=cv2.CV_32F)
disparity /= float(disparity.max())
cv2.imshow("STEREO", disparity)
cv2.waitKey(0)
cv2.imwrite("4_STEREO.png", disparity)

# replace the checkerboard in the image
chessboard = cv2.imread("images/img10.jpg")

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.1)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

gray = cv2.cvtColor(chessboard, cv2.COLOR_BGR2GRAY)

# find the chessboard corners
ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)
if ret:
  objpoints.append(objp)
  cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
  imgpoints.append(corners)

  # find the four corners of the checkerboard pattern
  color = (255, 0, 0)
  ul = corners[0]
  ur = corners[7]
  dl = corners[40]
  dr = corners[47]

  # compute the full matrix using linear mapping
  diff_left = (ul - dl) / 5.0
  diff_right = (ur - dr) / 5.0
  diff_up = (ur - ul) / 7.0
  diff_down = (dr - dl) / 7.0
  nul = ul - diff_up + diff_left
  ndl = dl - diff_down - diff_left
  nur = ur + diff_up + diff_right
  ndr = dr + diff_down - diff_right
  pul = (int(round(nul[0][0])),int(round(nul[0][1])))
  pdl = (int(round(ndl[0][0])),int(round(ndl[0][1])))
  pur = (int(round(nur[0][0])),int(round(nur[0][1])))
  pdr = (int(round(ndr[0][0])),int(round(ndr[0][1])))
  
  # load a sign and do homography transformation from one image to another
  sign = cv2.imread("helloworld.png")
  h,w = sign.shape[:2]
  src = np.float32([[[0, 0]], [[w, 0]], [[0, h]], [[w, h]]]).reshape(-1,1,2)
  dst = np.float32([pul, pur, pdl, pdr]).reshape(-1,1,2)
  homog, mask = cv2.findHomography(src, dst)
  ptrans = cv2.warpPerspective(sign, homog, (chessboard.shape[1],chessboard.shape[0]))
  np.copyto(chessboard, ptrans, "same_kind", ptrans != np.array([0,0,0]))

  # show the sign
  cv2.imshow("board", chessboard)
  cv2.waitKey(0)
  cv2.imwrite("EC.png", chessboard)
