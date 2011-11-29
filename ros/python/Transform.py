import os
import sys
import copy
import numpy as np
import scipy as sp

import roslib; roslib.load_manifest('scripting_util')
import rospy
import tf
import PyKDL as kdl

import geometry_msgs.msg as geometry_msgs

import RosUtil as ru

def I():
  ''' homogeneous transformation representing the identity '''
  return np.eye(4);


def trans(*args):
  ''' homogeneous transformation representing the translation t '''
  if len(args) > 1:
    t = (args[0], args[1], args[2]);
  else:
    t = args[0];

  return np.array([ [1, 0, 0, t[0]],
                    [0, 1, 0, t[1]],
                    [0, 0, 1, t[2]],
                    [0, 0, 0,    1] ]);


def rotx(t):
  ''' homogeneous transformation representing a rotation of theta about the x-axis '''
  ct = np.cos(t);
  st = np.sin(t);
  return np.array([ [1,  0,   0, 0],
                    [0, ct, -st, 0],
                    [0, st,  ct, 0],
                    [0,  0,   0, 1] ]);

                  
def roty(t):
  ''' homogeneous transformation representing a rotation of theta about the y-axis '''
  ct = np.cos(t);
  st = np.sin(t);
  return np.array([ [ ct,  0,  st, 0],
                    [  0,  1,   0, 0],
                    [-st,  0,  ct, 0],
                    [  0,  0,   0, 1] ]);


def rotz(t):
  ''' homogeneous transformation representing a rotation of theta about the z-axis '''
  ct = np.cos(t);
  st = np.sin(t);
  return np.array([ [ ct, -st, 0, 0],
                    [ st,  ct, 0, 0],
                    [  0,   0, 1, 0],
                    [  0,   0, 0, 1] ]);

def rot2tr(r):
  ''' creates the 4x4 transform matrix from a 3x3 rotation matrix '''
  R = np.eye(4);
  R[:3,:3] = r[:3,:3];

  return R;

def tr2rot(T):
  ''' extracts the rotation from the full transform '''
  R = np.eye(4);
  R[:3,:3] = T[:3,:3];

  return R;

def ezyx(*args):
  ''' 
    homogeneous transformation representing the euler ZYX angle 
    
    accepts either an iterable of the ZYX angles or 3 arguments Z,Y,X
  '''
  if len(args) > 1:
    zyx = (args[0], args[1], args[2]);
  else:
    zyx = args[0];

  return np.dot(rotz(zyx[0]), np.dot(roty(zyx[1]), rotx(zyx[2])));


def rosq2rot(q):
  ''' 
    returns the 4x4 rotation matrix representation of a unit quaternion 
    q is in ros pose messsage convention or a Quaternion message
    q = [x, y, z, w];
  '''
  if type(q) == geometry_msgs.Quaternion:
    return quat2rot([q.w, q.x, q.y, q.z]);
  else:
    return quat2rot([q[3], q[0], q[1], q[2]]);


def quat2rot(q):
  ''' 
    returns the 4x4 rotation matrix representation of a unit quaternion 
    q = [w, x, y, z];

    NOTE: this is not the ros pose convention [x, y, z, w]. use rosq2rot(q)
  '''
  q0 = q[0];
  q1 = q[1];
  q2 = q[2];
  q3 = q[3];
  return np.array([ [ q0**2 + q1**2 - q2**2 - q3**2,              2*(q1*q2 - q0*q3),             2*(q1*q3 + q0*q2),   0 ],
                    [             2*(q1*q2 + q0*q3),  q0**2 + q2**2 - q1**2 - q3**2,             2*(q2*q3 - q0*q1),   0 ],
                    [             2*(q1*q3 - q0*q2),              2*(q2*q3 + q0*q1), q0**2 + q3**2 - q1**2 - q2**2,   0 ],
                    [                             0,                              0,                             0,   1 ] ]);


def rospose2rot(pose):
  ps = copy.deepcopy(pose)
  if (type(ps) == geometry_msgs.PoseStamped):
    ps = ps.pose; 
  q = (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w);
  return rosq2rot(q);


def rospose2tr(pose):
  ''' returns a 4x4 transform representing the ros pose message '''
  ps = copy.deepcopy(pose)
  if (type(ps) == geometry_msgs.PoseStamped):
    ps = ps.pose; 
  p = (ps.position.x, ps.position.y, ps.position.z); 
  q = (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w); 
  return np.dot(trans(p), rosq2rot(q)); 
  

def rot2quat(r):
  ''' 
    creates a quaternion from a rotation 
    q = [w, x, y, z]

    NOTE: this is not the ros pose convention [x, y, z, w]. use rot2rosq(r) to get a ros quaternion message
  '''

  (x, y, z, w) = tf.transformations.quaternion_from_matrix(r);
  return np.array([w, x, y, z]);
  
  
def rot2rosq(r):
  ''' 
    converts a rot to ros quaternion message
  '''
  q = rot2quat(r);
  return geometry_msgs.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3]);


def tr2rospose(T):
  ''' creates a ros pose message from a transform '''
  return geometry_msgs.Pose(geometry_msgs.Point(*T[:3,3]), rot2rosq(T));


def tr2rot(R):
  '''
  pull out just the rotation part of a transformation matrix
  '''
  return np.array([ [R[0,0], R[0,1], R[0,2], 0],
                    [R[1,0], R[1,1], R[1,2], 0],
                    [R[2,0], R[2,1], R[2,2], 0],
                    [     0,      0,      0, R[3,3]]]);


def ev2rot(*args, **kwargs):
  ''' 
    converts eigen vectors to a 4x4 transform representing the rotation

    if eigen values are provided it will sort the vectors
    if no values are provided it is assumed the vectors are sorted

    accepted args:
    - iterable of 3 'vectors'
    - 3 vector arguments

    kwargs:
    - 'evals': iterable of 3 values

    returns the Quaternion as an array

    NOTE: this is not the ros pose convention [x, y, z, w]. use ev2rosq(r) to get a ros quaternion message
  ''' 
  if (len(args) == 1):
    evec = np.array(map(ru.to_array, args[0]));
  if (len(args) > 1):
    evec = np.array(map(ru.to_array, args[:3]));

  if (kwargs.has_key('evals')):
    # sort based on eigen values (high->low)
    isortEvals = np.argsort(ru.to_array(kwargs['evals']))[::-1];
    evec = evec[isortEvals];

  # normalize
  evec = np.array([(ev / np.linalg.norm(ev)) for ev in evec]);
  
  R = np.eye(4);
  R[:3,:3] = evec.T;

  return R;


def ev2quat(*args, **kwargs):
  ''' 
    converts eigen vectors to quaternion orientation 

    if eigen values are provided it will sort the vectors
    if no values are provided it is assumed the vectors are sorted

    accepted args:
    - iterable of 3 'vectors'
    - 3 vector arguments

    kwargs:
    - 'evals': iterable of 3 values

    returns the Quaternion as an array

    NOTE: this is not the ros pose convention [x, y, z, w]. use ev2rosq(r) to get a ros quaternion message
  ''' 
  R = ev2rot(*args, **kwargs);
  return rot2quat(R);

    
def ev2rosq(*args, **kwargs):
  ''' 
    converts eigen vectors to a ros quaternion message

    if eigen values are provided it will sort the vectors
    if no values are provided it is assumed the vectors are sorted

    accepted args:
    - iterable of 3 'vectors'
    - 3 vector arguments

    kwargs:
    - 'evals': iterable of 3 values

    returns the Quaternion message
  ''' 
  R = ev2rot(*args, **kwargs);
  return rot2rosq(R);


def rot2ev(R):
  '''
    converts a ros quaternion to eigen vectors 
  '''
  e1 = R[:3, 0];
  e2 = R[:3, 1];
  e3 = R[:3, 2];

  return (e1, e2, e3);


def rosq2ev(q):
  '''
    converts a ros quaternion to eigen vectors 
  '''
  R = rosq2rot(q);
  return rot2ev(R);


def quat2ev(q):
  '''
    converts a ros quaternion to eigen vectors 
    q = [w, x, y, z];

    NOTE: this is not the ros pose convention [x, y, z, w]. use rosq2rot(q)
  '''
  R = quat2rot(q);
  return rot2ev(R);


def axis_angle2rosq(axis, angle):
  '''
    converts axis angle rotation representation to ros quaternion
  '''
  (q0, q1, q2, q3) = axis_angle2quat(axis, angle);
  return geometry_msgs.Quaternion(w=q0, x=q1, y=q2, z=q3);


def axis_angle2quat(axis, angle):
  '''
    converts axis angle rotation representation to quaternion
    return [w, x, y, z]

    NOTE: this is not the ros pose convention [x, y, z, w]. use rosq2rot(q)
  '''
  q0 = np.cos(angle/2.0); 
  (q1, q2, q3) = np.sin(angle/2.0) * np.array(axis);
  return np.array([q0, q1, q2, q3]);


def axis_angle2rot(axis, angle):
  '''
    converts axis angle rotation representation to ros quaternion
  '''
  return quat2rot(axis_angle2quat(axis, angle));



def translate_rospose(pose, t):
  ''' 
    translates a ros pose with respect to the poses orientation

    return will be a Pose or PoseStamped based on the input (header will be copied)
  '''
  t = ru.to_array(t); 

  if (t.ndim == 2):
    # 4x4 transform (extract the translation from it) 
    t = trans(t[:3,3]);
  else:
    t = trans(t[:3]);

  T = rospose2tr(pose);
  nT = np.dot(T, t);

  if (type(pose) == geometry_msgs.PoseStamped):
    return geometry_msgs.PoseStamped(pose=tr2rospose(nT), header=copy.copy(pose.header));
  else:
    return tr2rospose(nT);


def rotate_rospose(pose, r):
  ''' 
    rotates a ros pose orientation with respect to the poses current orientation
    rot: either list of EulerZYX angles (z, y, x) or 4x4 transform matrix

    return will be a Pose or PoseStamped based on the input (header will be copied)
  '''
  R = ru.to_array(r); 

  if (R.ndim == 2):
    # 4x4 transform (extract the rotation from it) 
    R[:3,3] = 0.0;
  else:
    R = ezyx(r);

  T = rospose2tr(pose);
  nT = np.dot(T, R);

  if (type(pose) == geometry_msgs.PoseStamped):
    return geometry_msgs.PoseStamped(pose=tr2rospose(nT), header=copy.copy(pose.header));
  else:
    return tr2rospose(nT);


def Tdist(T0, T1, transWeight=1.0, rotWeight=1.0, method=2):
  if (method == 1):
    # 1 norm
    td = np.linalg.norm(T0[:3,3] - T1[:3,3])
    rd = np.linalg.norm(rot2quat(T0) - rot2quat(T1), ord=1);
    return (transWeight * td + rotWeight * rd);
  
  if (method == 2):
    # 2 norm
    td = np.linalg.norm(T0[:3,3] - T1[:3,3])
    rd = np.linalg.norm(rot2quat(T0) - rot2quat(T1));
    return (transWeight * td + rotWeight * rd);

  if (method == 'ax'):
    # squared angle between each axis
    td = np.linalg.norm(T0[:3,3] - T1[:3,3])
    (x0, y0, z0) = rot2ev(T0);
    (x1, y1, z1) = rot2ev(T1);
    rd = np.linalg.norm([ru.angle_between(v0, v1) for (v0, v1) in ((x0, x1), (y0, y1), (z0, z1))]);
    return (transWeight * td + rotWeight * rd);

def order_transforms(Tlist, Tbase, transWeight=1.0, rotWeight=1.0, method=2):
  ''' orders the Transforms/Frames '''
  Tlist = [ru.to_array(T) for T in Tlist]; 
  Tbase = ru.to_array(Tbase);

  d = [Tdist(T, Tbase, transWeight, rotWeight, method) for T in Tlist];

  return (np.argsort(d), d); 

