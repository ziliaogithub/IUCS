#!/usr/bin/env python  

import os
import sys
import time
import copy
import string
import numpy as np
import scipy as sp


def angle_between(a, b):
  '''
    finds the angle between the given vectors in radians
  '''
  return np.arctan2(np.linalg.norm(np.cross(a,b)), np.dot(a,b));


def line_line_closest_point(p1, n1, p2, n2):
  ''' 
    computes the closest point of intersection from the point/normal pairs 
    
    solve the system for (s,t):
    (n1.n1)*s - (n1.n2)*t = n1.(p2-p1)
    (n1.n2)*s - (n2.n2)*t = n2.(p2-p1)

    return: (p1 + s*n1, p2 + t*n2);
  '''
  p1 = np.array(p1);
  p2 = np.array(p2);

  A = np.array([[np.dot(n1, n1), -np.dot(n1, n2)],
                [np.dot(n1, n2), -np.dot(n2, n2)]]);

  try:
    Ainv = np.linalg.inv(A);
  except np.linalg.LinAlgError, err:
    rospy.logwarn(__name__ + ': line_line_closest_point: lines are parallel');
    # parrallel lines
    return (p1, p2);

  (s,t) = np.dot(Ainv, np.array([np.dot(n1, (p2-p1)), np.dot(n2, (p2-p1))]));

  return (p1 + s*n1, p2 + t*n2);


def line_plane_intersection(lp, ln, pp, pn):
  '''
    computes the point intersection of the line and plane from the point/normal pairs
    lp,ln - parameterized form of the line (point, dir) p = s*ln + lp

    s = (pp - lp).pn / (ln.pn)

    return: (intersection)
  '''
  lp = np.array(lp);
  ln = np.array(ln);
  pp = np.array(pp);
  pn = np.array(pn);

  num = np.dot((pp - lp), pn);
  dem = np.dot(ln, pn);

  if (np.abs(dem) < 1e-10):
    if (np.abs(num) < 1e-10):
      # line is on the plane
      return (pp);
    else:
      # they are parallel
      rospy.logwarn(__name__ + ': line_plane_intersection: line/plane are parallel');
      return (pp);

  return (lp + (num/dem)*ln);


def plane_plane_intersection(p1, n1, p2, n2):
  ''' 
    computes the line intersection of the two planes from the point/normal pairs
    (from Gellert et al. 1982)

    # in hessian normal form (n.x = -p)
    A = [n1; n2]
    b = [p1; p2]
    A.p = b

    return the parameterized line form
    return: (point, direction)
  '''
  p1 = np.array(p1);
  n1 = np.array(n1);
  p2 = np.array(p2);
  n2 = np.array(n2);

  # direction is the cross of the normals
  a = np.cross(n1, n2);
  a /= np.linalg.norm(a);

  A = np.array([n1, n2]);
  b = np.array([np.linalg.norm(p1), np.linalg.norm(p2)]);
  try:
    Ainv = np.linalg.pinv(A);
  except np.linalg.LinAlgError, err:
    rospy.logwarn(__name__ + ': plane_plane_intersection: planes are parallel. %s' % err);
    # parrallel planes
    return (p1, a);
  
  p = np.dot(Ainv, b);

  return (p, a);
