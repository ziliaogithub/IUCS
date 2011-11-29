import os
import sys
import time
import copy
import math
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt 

import roslib; roslib.load_manifest('scripting_util');
import rospy
import tf
import PyKDL as kdl
import cv

import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs

import RosUtil as ru
import Transform as tr
import PointCloudUtil as pcu

import ObjectModel as om

np.set_printoptions(suppress=True)
np.set_printoptions(precision=3)


def distance_matrix(a, b):
  ''' 
    returns the distance matrix for two sets of points
    using euclidean distance d = abs(sum((a-b)**2))
    
    a and b are DxN matrixes 
  '''

  aa = np.array([np.sum(a**2, axis=0)]);
  bb = np.array([np.sum(b**2, axis=0)]);
  ab = np.dot(a.T, b);

  #d = np.sqrt(np.abs(np.repeat(aa.T, bb.shape[1], axis=1) + np.repeat(bb, aa.shape[1], axis=0) - 2*ab));
  d = np.abs(np.repeat(aa.T, bb.shape[1], axis=1) + np.repeat(bb, aa.shape[1], axis=0) - 2*ab);

  return d;


def closest_points(P, X):
  ''' 
    computes the closest points from the two data sets 

    matrix: DxN
  '''
  d = distance_matrix(P, X);

  imin = np.argmin(d, axis=1);

  return (imin, d);

def compute_registration(P, X):
  '''
    compute the registration between the two data sets
    Pi should correspond to Xi (distance)

    matrix: 4xN

    return:
      qr - quaterion rotation
      qt - translation
  '''

  # compute mean
  uP = np.array([np.sum(P, axis=1)/P.shape[1]]);
  uX = np.array([np.sum(X, axis=1)/X.shape[1]]);

  # cross-covariance matrix
  cc = np.dot(P[:3,:], X[:3,:].T)/P.shape[1] - np.dot(uP[:,:3].T, uX[:,:3]);

  A = cc - cc.T;
  delta = np.array([[A[1,2], A[2,0], A[0,1]]]).T;

  trace = np.trace(cc);
  t0 = np.hstack(([[trace]], delta.T));
  t1 = np.hstack((delta, cc + cc.T - trace * np.eye(3)));
  Q = np.vstack((t0, t1));

  # comute quaternion
  # main eigenvector of Q
  (U, s, Vh) = np.linalg.svd(Q);
  qr = Vh[0];

  # compute translation 
  R  = tr.quat2rot(qr);
  qt = uX - np.dot(R,uP.T).T;

  return (qr, qt[0,:3]);


def icp(data, model, thres=.001, maxIter=1000):
  '''
    compute icp on two point sets

    matrix: DxN

    return:
      qr - quaterion rotation
      qt - translation
  '''
  # augment if needed
  if data.shape[0] == 3:
    data = np.vstack((data, np.ones(data.shape[1])));
  if model.shape[0] == 3:
    model = np.vstack((model, np.ones(model.shape[1])));

  # initialize registration
  qr = np.array([1.0, 0.0, 0.0, 0.0]);
  qt = np.array([0.0, 0.0, 0.0]);
  dm = np.Inf;
  count = 0;

  # compute initial closest points
  di = np.arange(data.shape[1]);
  (imin, d) = closest_points(data, model);
  dk = np.sum(d[di, imin]);

  # loop until the change in error is below the threshold
  while (dm > thres and count < maxIter):
  
    # compute registration
    (qr, qt) = compute_registration(data, model[:,imin]);

    # transform data
    T = np.dot(tr.trans(qt), tr.quat2rot(qr));
    tdata = np.dot(T, data);

    # compute closest points
    (imin, d) = closest_points(tdata, model);
    dk1 = np.sum(d[di, imin]);

    dm = np.abs(dk1 - dk);
    dk = dk1;
    count += 1;

  return (qr, qt, dk);

