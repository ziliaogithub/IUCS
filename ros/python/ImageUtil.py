# new ImageUtil that does not rely on cv

import sys
import time
import math
import copy 
import string
import numpy as np
import scipy as sp

import roslib; roslib.load_manifest('ros_python_util')
import rospy
import tf
import cv
import cv_bridge
import image_geometry

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs

import Transform as tr
import RosUtil as ru

# setup publishers
rospy.logdebug(__name__ + ': setting up publishers...');
chessboardDetectionPub = rospy.Publisher('/chessboard_detection', sensor_msgs.Image);


def msg2cvmat(image):
  ''' converts a ros Image msg to cv image '''
  bridge = cv_bridge.CvBridge();
  return bridge.imgmsg_to_cv(image);


def cvmat2msg(image, encoding='passthrough', stamp=rospy.Time(), frame_id=''):
  ''' converts a cv image to ros Image msg '''
  bridge = cv_bridge.CvBridge();
  msg = bridge.cv_to_imgmsg(image, encoding);
  # the bridge does not calculate step correctly
  if (type(image) == cv.cvmat):
    msg.step = image.step;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  return msg;


def info2cvmat(info):
  ''' 
    converts a ros CameraInfo msg to cv mats 
    return: (cameraMatrix, distCoeffs, projectionMatrix, rotationMatrix)
  '''
  phc = image_geometry.PinholeCameraModel();
  phc.fromCameraInfo(info);

  cameraMatrix = phc.intrinsicMatrix();
  distCoeffs = phc.distortionCoeffs();
  projectionMatrix = phc.projectionMatrix();
  rotationMatrix = phc.rotationMatrix();

  return (cameraMatrix, distCoeffs, projectionMatrix, rotationMatrix); 


def info2array(info):
  ''' 
    converts a ros CameraInfo msg to cv mats 
    return: (cameraMatrix, distCoeffs, projectionMatrix, rotationMatrix)
  '''
  ret = info2cvmat(info);

  return tuple(map(np.asarray, ret));


def rgb2gray(img, method='luminosity'):
  '''
    converts the rgb image (MxNx3) array to grayscale

    methods available:
      luminosity: 0.21 * R + 0.71 * G + 0.07 * B
      average:    (R + G + B) / 3.0
      lightness:  (max(R, G, B) + min(R, G, B)) / 2.0
  '''
  img = img.astype('f');
  R = img[:,:,0];
  G = img[:,:,1];
  B = img[:,:,2];

  if (method == 'luminosity'):
    return (0.21 * R) + (0.71 * G) + (0.07 * B);
  elif (method == 'average'):
    return (R + G + B) / 3.0;
  elif (method == 'lightness'):
    return (np.max(img, axis=2) + np.min(img, axis=2)) / 2.0;
  else:
    rospy.logfatal(__name__ + ': rgb2gray: unkown conversion method: %s' % method);


def project_points(points, caminfo):
  ''' 
    projects the 3D point into the desired image coordinates 

    points: array of points to project (default is Nx3) in the camera frame

    return: Nx2 array of projected points
  '''
  # format input correctly
  points = ru.to_array(points);
  pshape = points.shape;
  if (len(pshape) < 2):
    points = np.array([points]);
  elif (max(pshape) > 3):
    if (pshape[1] > pshape[0]):
      points = points.T; 
  elif (max(pshape) == 3):
    if (pshape[0] > pshape[1]):
      points = points.T;

  # get camera matrix
  C = np.reshape(np.array(caminfo.K), (3,3));
  # get distorition coef
  (k1, k2, p1, p2, k3) = caminfo.D;  
    
  # normalize with z
  points = points.T/points[:,2];

  # undistort
  r2 = np.sum(points[:2,:]**2, axis=0);
  xy = np.prod(points[:2,:], axis=0);
  d1 = (1.0 + k1*r2 + k2*r2**2 + k3*r2**3);
  c1 = points[:2,:]*np.array([d1,d1]);
  c2 = np.array([2.0*p1*xy, 2.0*p2*xy]);
  c3 = np.array([p2*(r2 + 2*points[0,:]**2), p1*(r2 + 2*points[1,:]**2)]);
  points[:2,:] = c1 + c2 + c3;

  return C;

  # project
  prj =  np.dot(C, points);

  if (prj.shape[1] == 1):
    return prj[:2,0].T;
  else:
    return prj[:2,:].T;


def find_chessboard(img, ncol=5, nrow=4, useSubPix=True):
  '''   
    wrapper for the open cv chessboard detection
    img: color sensor_msgs.Image or cvmat of the image image
  '''
  # convert image msg to cv
  if (type(img) == sensor_msgs.Image):
    img = msg2cvmat(img);
  
  # create chessboard size tuple
  chessboardSize = (ncol, nrow);
  
  # find corners
  (success, chessboard) = cv.FindChessboardCorners(img, chessboardSize, flags=cv.CV_CALIB_CB_ADAPTIVE_THRESH + \
                                                   cv.CV_CALIB_CB_NORMALIZE_IMAGE + cv.CV_CALIB_CB_FILTER_QUADS)

  if (success != 0):
    if (useSubPix): 
      # create grayscale
      gray = cv.CreateImage((img.cols, img.rows), cv.IPL_DEPTH_8U, 1);
      cv.CvtColor(img, gray, cv.CV_RGB2GRAY);

      # compute window (1/2 distance betweeen two corners)
      c0 = np.array(chessboard[0]);
      c1 = np.array(chessboard[1]);
      w = int(np.ceil(np.sqrt(np.dot((c1 - c0), (c1 - c0)))/2.0));

      # sub pixel refinement
      chessboard = cv.FindCornerSubPix(gray, chessboard, (w,w), (-1,-1),
                                       (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 20, 0.01))


  return (success, np.array(chessboard));


def draw_chessboard(img, chessboard, ncol=5, nrow=4):
  '''
    draws the chessboard detection from find_chessboard on the image
  '''
  # convert image msg to cv
  if (type(img) == sensor_msgs.Image):
    img = msg2cvmat(img);

  chessboard = tuple(map(tuple, chessboard));

  cv.DrawChessboardCorners(img, (ncol, nrow), chessboard, True);

  return img;


def get_chessboard_pose(img, info, ncol=5, nrow=4, squareSize=0.045, useSubPix=True, publishDetection=True, isRect=True):
  ''' 
    compute the pose of the chessboard in the camera frame   
  '''
  (success, chessboard_wrtImage) =  find_chessboard(img, ncol=ncol, nrow=nrow, useSubPix=useSubPix);
  if (len(chessboard_wrtImage) == 0):
    rospy.logwarn(__name__ + ': no chessboard found');
    return None;

  if (publishDetection):
    chessboardCV = draw_chessboard(img, chessboard_wrtImage, ncol=ncol, nrow=nrow);
    chessboardMsg = cvmat2msg(chessboardCV, img.encoding, frame_id=img.header.frame_id, stamp=rospy.Time().now());
    chessboardDetectionPub.publish(chessboardMsg);

  # compute pose
  chessboard_wrtBoard = cv.CreateMat(ncol*nrow, 1, cv.CV_32FC3);
  for i in range(ncol*nrow):
    chessboard_wrtBoard[i,0] = ((i % ncol) * squareSize, (i / ncol) * squareSize, 0);

  # get camera intrinsics
  (cameraMatrix, distCoeffs, projectionMatrix, rotationMatrix) = info2cvmat(info);

  # extrinsic outputs 
  rot = cv.CreateMat(3, 1, cv.CV_32FC1)
  trans = cv.CreateMat(3, 1, cv.CV_32FC1)

  # find chessboard pose
  if isRect:
    distCoeffs = cv.CreateMat(1, 4, cv.CV_32FC1);
    distCoeffs[0,0] = 0; distCoeffs[0,1] = 0; distCoeffs[0,2] = 0; distCoeffs[0,3] = 0;
    cv.FindExtrinsicCameraParams2(chessboard_wrtBoard, chessboard_wrtImage, projectionMatrix[:,:3], distCoeffs, rot, trans);
  else:
    cv.FindExtrinsicCameraParams2(chessboard_wrtBoard, chessboard_wrtImage, cameraMatrix, distCoeffs, rot, trans);

  # get transform from rot, trans
  rot = np.asarray(rot);
  trans = np.asarray(trans);

  th = np.linalg.norm(rot);
  r = rot / th;
  R = np.cos(th)*np.eye(3) + (1 - np.cos(th))*np.dot(r, r.T) + np.sin(th)*np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]]);

  if not isRect:
    Trect = tr.trans(-projectionMatrix[0,3]/cameraMatrix[0,0],-projectionMatrix[1,3]/cameraMatrix[1,1], 0)
    Rcam = tr.rot2tr(np.asarray(rotationMatrix));
    Tmodel2cam = np.dot(Trect, np.dot(Rcam, np.dot(tr.trans(trans), tr.rot2tr(R))));
  else:
    Tmodel2cam = np.dot(tr.trans(trans), tr.rot2tr(R));

  return ru.to_PoseStamped(Tmodel2cam, frame_id=info.header.frame_id, stamp=info.header.stamp); 

