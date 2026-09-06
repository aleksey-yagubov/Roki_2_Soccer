import numpy as np
import cv2
import glob
import json

# Define the chess board rows and columns
CHECKERBOARD = (6,9)
subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
calibration_flags = cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC 
#calibration_flags = cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_USE_EXTRINSIC_GUESS
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
counter = 0
for path in glob.glob('*.png'):
    # Load the image and convert it to gray scale
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
    # Make sure the chess board pattern was found in the image
    if ret:
        objpoints.append(objp)
        corners = cv2.cornerSubPix(gray,corners,(20,5),(-1,-5),subpix_criteria)
        imgpoints.append(corners)
        #cv2.drawChessboardCorners(img, (rows, cols), corners, ret)
    print(str(path)) 
    counter+=1

N_imm = counter# number of calibration images
img_size = gray.shape[::-1]
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
print('K :', K)
rms, K, _, _, _ = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    img_size,
    K,
    D,
    rvecs,
    tvecs,
    calibration_flags,
    (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
print('K :', K)
img =cv2.imread("sample1.png")
new_size = (1600,1300)
P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, img_size, np.eye(3), balance = 0.0, new_size = new_size, fov_scale = 2.0)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), P, new_size, cv2.CV_16SC2)
np.save("Camera_calibration_K", K)
np.save("Camera_calibration_D", D)
np.save("Camera_calibration_P", P)
np.save("Camera_calibration_map1", map1)
np.save("Camera_calibration_map2", map2)

################################################################
map1 = np.load('Camera_calibration_map1.npy')
map2 = np.load('Camera_calibration_map2.npy')
undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
undistorted_img3 = undistorted_img[325:975, 400:1200]

undistorted_img = cv2.resize(undistorted_img, (800,650))
undistorted_img3 = cv2.resize(undistorted_img3, (800,650))
cv2.imshow('Original Image', cv2.resize(img, (800,650)))
cv2.imshow('Undistort Image', undistorted_img)
cv2.imshow('Undistort Image3', undistorted_img3)
cv2.waitKey(0)