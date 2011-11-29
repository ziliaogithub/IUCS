import sys
import time
import math
import copy 
import string
import struct
import random
import numpy as np
import scipy as sp

import roslib; roslib.load_manifest('scripting_util')
import rospy

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs

import RosUtil as ru
import MathUtil as mu
import Transform as tr

PC2_DATATYPES = ('', 'i1', 'u1', 'i2', 'u2', 'i4', 'u4', 'f4', 'f8');
NUMPY_2_PC2_DATATYPE = { '':0, 'int8':1, 'uint8':2, 'int16':3, 'uint16':4, 'int32':5, 'uint32':6, 'float32':7, 'float64':8 };


def to_PointCloud2(*args, **kwargs):
  '''
    converts the input to a sensor_msgs/PointCloud2 message

    supported input:
      - ROS PointCloud2 messages
      - numpy structured array
      
      NYI:
        - ROS PointCloud messages
        - iterable of ROS Point messages
        - iterable of kdl Vectors
        - iterable of iterables
        - numpy array Nx3 or 3xN (if N == 3 then it is assumed Nx3)

    keyword args:
      - frame_id 
      - stamp

    NOTE: mixing types is not supported
  '''

  if (len(args) > 0 and type(args[0]) == sensor_msgs.PointCloud2):
    return copy.deepcopy(args[0]);

  pc2 = sensor_msgs.PointCloud2();

  # assumed to be a numpy structured array 
  pts = args[0];
  (pc2.fields, ptstep) = get_point_fields(pts.dtype);

  if (len(pts.shape) == 1):
    pc2.height = 1;
    pc2.width = pts.shape[0];
  else:
    pc2.height = pts.shape[0];
    pc2.width = pts.shape[1];

  pc2.is_bigendian = False;
  pc2.point_step = ptstep;
  pc2.row_step = ptstep * pc2.width;
  pc2.data = args[0].tostring();

  pc2.is_dense = True;

  if ('frame_id' in kwargs.keys()):
    pc2.header.frame_id = kwargs['frame_id'];

  if ('stamp' in kwargs.keys()):
    pc2.header.stamp = kwargs['stamp'];

  return pc2; 


def to_PointCloud2XYZ(*args, **kwargs):
  '''
    converts the input to a sensor_msgs/PointCloud2 message with fields x,y,z

    supported input:
      - numpy array Nx3 or 3xN (if N == 3 then it is assumed Nx3)
      
      NYI:
        - ROS PointCloud messages
        - iterable of ROS Point messages
        - iterable of kdl Vectors
        - iterable of iterables
        - numpy array Nx3 or 3xN (if N == 3 then it is assumed Nx3)
        - numpy array Nx3 or 3xN (if N == 3 then it is assumed Nx3)

    keyword args:
      - frame_id 
      - stamp

    NOTE: mixing types is not supported
  '''

  pc2 = sensor_msgs.PointCloud2();

  # assumed to be a numpy array 
  pts = args[0];
  if (type(pts) == np.ndarray):
    # 3xN (N > 3)
    if (max(pts.shape) > 3 and pts.shape[1] > pts.shape[0]):
      pts = pts.T;
    # Nx3 or 3xN (N < 3)
    elif (pts.shape != (3,3) and pts.shape[0] == 3):
      pts = pts.T;

    # pts should be Nx3 now
    pc2.height = 1;
    pc2.width = pts.shape[0];

    # format fields
    pc2dtype = NUMPY_2_PC2_DATATYPE[pts.dtype.name];
    offset = pts.dtype.itemsize;
    pc2.fields = [sensor_msgs.PointField('x', 0, pc2dtype, 1),
                  sensor_msgs.PointField('y', offset, pc2dtype, 1),
                  sensor_msgs.PointField('z', 2*offset, pc2dtype, 1)];
    pc2.is_bigendian = False;

    pc2.point_step  = 3*offset;
    pc2.row_step = pc2.width * pc2.point_step;

    pc2.data = pts.tostring();
    pc2.is_dense = True;

  if ('frame_id' in kwargs.keys()):
    pc2.header.frame_id = kwargs['frame_id'];

  if ('stamp' in kwargs.keys()):
    pc2.header.stamp = kwargs['stamp'];

  return pc2; 


def from_PointCloud2(pc2):
  '''
    converts a PointCloud2 message to a structured numpy array
  '''
  # get point struct format
  fmt = get_field_fmt(pc2.fields, pc2.is_bigendian);

  # convert to numpy ndarray
  if (pc2.width == 0 and pc2.height == 0):
    a = np.array([], dtype=fmt);
  else:
    a = np.fromstring(pc2.data, dtype=fmt, count=pc2.width*pc2.height);

  # reshape for width/height
  return a.reshape((pc2.height, pc2.width));


def get_field_fmt(fields, isBigEndian):
  '''
    formats the numpy datatype string from the field list
  '''
  bo = '>' if isBigEndian else '<';
 
  # sort fields by offset
  sfields = sorted(fields, key=lambda f: f.offset);

  # create field format tuples
  return map(lambda f: (f.name, bo+PC2_DATATYPES[f.datatype], f.count), sfields);


def get_point_fields(dtype):
  '''
    creates a PointField from a numpy dtype tuple list
  '''
  fields = [];
  offset = 0;
  for fname in dtype.names:
    pf = sensor_msgs.PointField();
    pf.name = fname;
    pf.datatype = NUMPY_2_PC2_DATATYPE[dtype[fname].name];
    pf.count = np.prod(dtype[fname].shape) if dtype[fname].shape else 1;
    pf.offset = offset;
    fields.append(pf);

    # update offset
    offset += dtype[fname].itemsize * pf.count;

  # return the fields and the point step (offset)
  return (fields, offset);


def structured_array2ndarray(arr):
  '''
    converts the structured array to an flattened ndarray 
    assumes all fields are the same datatype
    return array is NxD where each column is the flattened field
  '''
  dtypes = arr.dtype;
  flatarr = arr.flatten();
  narr = np.empty((len(flatarr), len(dtypes)), dtype=dtypes[0]);
  i = 0;
  for name in dtypes.names:
    narr[:,i] = flatarr[name];
    i += 1;

  return narr;


def extract_indicies(arr, ind):
  '''
    extracts the indicies from the given array

    assumes either Nx3 or 3xN
  '''
  ind = np.array(ind);
  if (max(arr.shape) > 3 and arr.shape[1] > arr.shape[0]
      or (arr.shape != (3,3) and arr.shape[0] == 3)):
    # input is 3xN
    inliers = np.array([False]*arr.shape[1]);
    inliers[ind] = True;
    return (arr[:,inliers], arr[:,inliers==False]);
  else:
    inliers = np.array([False]*arr.shape[0]);
    inliers[ind] = True;
    return (arr[inliers,:], arr[inliers==False,:]);


def get_bbox(pc, pose=None):
  '''
    get the bounding box for the point cloud
    if a pose is provided then it will find the bbox
      for the data aligned to that pose

    return: bbox array
        |xmin, xmax|
        |ymin, ymax|
        |zmin, zmax|
  '''
  pts = ru.to_array(pc);

  if pose != None:
    ps = ru.to_PoseStamped(pose);
    ps_wrtPC = ru.transformPose(ru.get_frame_id(pc), ps);

    T = tr.rospose2tr(ps_wrtPC.pose);
    pts = np.dot(np.linalg.inv(T), np.vstack((pts.T, np.ones(pts.shape[0])))).T; 

  pmin = np.min(pts[:,:3], axis=0);
  pmax = np.max(pts[:,:3], axis=0);
  bbox = np.array([pmin, pmax]).T;

  return bbox;


def get_centroid(pts):
  '''
    computes the centroid of the point cloud
  '''
  pts = ru.to_array(pts);

  return np.mean(pts, axis=0);


def get_ev(pts, includeEigenValues=False):
  '''
    computes the eigen vectors for the point cloud
    arr is expected to be Nx3

    return:
      array of row vectors (e1, e2, e3)
  '''
  pts = ru.to_array(pts);

  trans = -1 * np.mean(pts, axis=0);
  pts += trans;

  cov = np.cov(pts.T);
  (u, s, vh) = np.linalg.svd(cov);

  if includeEigenValues:
    return (vh, s);
  else:
    return vh;


def get_blob_pose(pts):
  '''
    computes the pose of the point cloud
    arr is expected to be Nx3

    return Pose
  '''
  pts = ru.to_array(pts);
  
  cent = get_centroid(pts);
  (x, y, z) = get_ev(pts);
  
  pose = geometry_msgs.Pose();
  pose.position = geometry_msgs.Point(*cent);
  pose.orientation = tr.ev2rosq((x,y,z));

  return pose;


def estimate_normals(pts, k=5, weightNeighbors=True, viewpoint=None):
  '''
    estimates the point cloud normals from an unorder point cloud
    pts: NxD point array
    k: number of nearest neighbors to consider
    weightNeighbors: flag indicating that weights on the neighbors should be used
    viewpoint: vector indicating the viewpoint direction to correct direction
  '''
  def compute_normal(pts):
    cent = np.mean(pts);
    cov = np.cov(np.transpose(pts - cent));
    (u, s, vh) = np.linalg.svd(cov);
    return vh[2,:];

  pts = ru.to_array(pts);
  
  # compute distance matrix
  d = distance_matrix(pts, pts);
  # find nearest neighbors
  nn = np.argsort(d, axis=1);

  normals = np.array(map(lambda row: compute_normal(pts[row[:k+1],:]), nn));

  if (viewpoint != None):
    vcheck = map(lambda i: np.dot(normals[i,:], viewpoint - pts[i,:]), np.arange(pts.shape[0]));
    normals[(vcheck < 0),:] *= -1.0;

  return normals;


def distance_matrix(a, b):
  ''' 
    returns the distance matrix for two sets of points
    using euclidean distance d = sqrt(sum((a-b)**2))
    
    a and b are NxD matrixes 
  '''
  a = a.T;
  b = b.T;

  aa = np.array([np.sum(a**2, axis=0)]);
  bb = np.array([np.sum(b**2, axis=0)]);
  ab = np.dot(a.T, b);

  d = np.sqrt(np.repeat(aa.T, bb.shape[1], axis=1) + np.repeat(bb, aa.shape[1], axis=0) - 2*ab);

  return d;


def create_normals_pose_array(pts, normals, frame_id=''):
  ''' 
    creates a pose array for visualizing the normals 
    pts: Nx3 array of points
    normals: Nx3 array of corresponding normals
  '''
  def compute_pose(c, n):
    v = np.cross([1, 0, 0], n);
    v /= np.linalg.norm(v);
    th = ru.angle_between([1, 0, 0], n);
    return geometry_msgs.Pose(geometry_msgs.Point(*c), tr.axis_angle2rosq(v, th));

  pa = geometry_msgs.PoseArray();
  pa.header.frame_id = frame_id;
  pa.poses = map(lambda i: compute_pose(pts[i,:], normals[i,:]), np.arange(pts.shape[0]));
  return pa;


def fit_plane(p1, n1, p2, n2, p3, n3, alpha=np.deg2rad(30.0)):
  ''' 
    fit a plane to the three point/normal pairs 
    alpha: angle thres for determining if the fit is a valid sphere
          both normals must be < alpha from the sphere normals

    return (valid, p, normal);
      valid is a bool indicating if the fit was valid based on the points and normals
  '''
  p1 = np.asarray(p1);
  n1 = np.asarray(n1);
  p2 = np.asarray(p2);
  n2 = np.asarray(n2);
  p3 = np.asarray(p3);
  n3 = np.asarray(n3);

  v1 = p2 - p1;
  v2 = p3 - p1;
  n = np.cross(v1, v2);
  n /= np.linalg.norm(n);

  sa = np.sin(alpha);
  sang = map(lambda nx: np.sin(np.abs(ru.angle_between(n, nx))), [n1, n2, n3]);
  if (np.any(sang > sa)):
    return (False, p1, n);
  return (True, p1, n);


def fit_sphere(p1, n1, p2, n2, eps=0.005, alpha=np.deg2rad(30.0)):
  ''' 
    fit a sphere to the two point/normal pairs 
    eps: distance thres for determining if the fit is a valid sphere
          both points must be < eps from the sphere
    alpha: angle thres for determining if the fit is a valid sphere
          both normals must be < alpha from the sphere normals

    return (valid, center, radius);
      valid is a bool indicating if the fit was valid based on the points and normals
  '''
  p1 = np.asarray(p1);
  n1 = np.asarray(n1);
  p2 = np.asarray(p2);
  n2 = np.asarray(n2);

  # find closes points on the lines
  (pc0, pc1) = mu.line_line_closest_point(p1, n1, p2, n2);
  # center of the sphere
  c = (pc0 + pc1) / 2.0;
  # compute radius
  r = (np.linalg.norm(p1-c) + np.linalg.norm(p2-c))/2.0;

  # check if the fit is valid
  if ((np.abs(r - np.linalg.norm(p1 - c)) > eps)
      or (np.abs(r - np.linalg.norm(p2 - c)) > eps)):
    return (False, c, r);

  sa = np.sin(alpha);
  if ((np.sin(np.abs(ru.angle_between(n1, p1-c))) > sa)
      or (np.sin(np.abs(ru.angle_between(n2, p2-c))) > sa)):
    return (False, c, r);
  
  return (True, c, r);


def fit_cylinder(p1, n1, p2, n2, eps=0.005, alpha=np.deg2rad(10.0)):
  ''' 
    fit a cylinder to the two point/normal pairs 
    eps: distance thres for determining if the fit is a valid cylinder
          both points must be < eps from the cylinder
    alpha: angle thres for determining if the fit is a valid cylinder
          both normals must be < alpha from the cylinder normals

    return (valid, center, axis, radius);
      valid is a bool indicating if the fit was valid based on the points and normals
  '''
  p1 = np.asarray(p1);
  n1 = np.asarray(n1);
  p2 = np.asarray(p2);
  n2 = np.asarray(n2);

  # find cylinder axis
  a = np.cross(n1, n2);
  a /= np.linalg.norm(a);

  # project lines (defined by the point/normal) onto the plane defined by a.x = 0
  pp1 = p1 - a * np.dot(a, p1);
  pn1 = n1 - a * np.dot(a, n1);
  pp2 = p2 - a * np.dot(a, p2);
  pn2 = n2 - a * np.dot(a, n2);
  
  # find intersection
  (c, c1) = mu.line_line_closest_point(pp1, pn1, pp2, pn2);

  # cylinder radius
  r = np.linalg.norm(pp1 - c);

  # check if the fit is valid
  
  # find rejections of the points from the cylinder axis
  cp1 = pp1 - c;
  cp2 = pp2 - c;
  ca = c + a;
  rej1 = cp1 - np.dot(cp1, ca)*ca; 
  rej2 = cp2 - np.dot(cp2, ca)*ca; 

  pa = create_normals_pose_array(np.array([p1,p2]), np.array([rej1,rej2]));
  #Debug.pa1Pub.publish(pa);
  print rej1
  print rej2
  print np.sin(np.abs(ru.angle_between(rej1, n1)))
  print np.sin(np.abs(ru.angle_between(rej2, n2)))
  print np.abs(r - np.linalg.norm(rej1))
  print np.abs(r - np.linalg.norm(rej2))


  sa = np.sin(alpha);
  if (((np.sin(np.abs(ru.angle_between(rej1, n1)))) > sa)
      or (np.sin(np.abs(ru.angle_between(rej2, n2))) > sa)):
    return (False, c, a, r);

  if ((np.abs(r - np.linalg.norm(rej1)) > eps)
      or (np.abs(r - np.linalg.norm(rej2)) > eps)):
    return (False, c, a, r);

  sa = np.sin(alpha);
  if (((np.sin(np.abs(ru.angle_between(rej1, n1)))) > sa)
      or (np.sin(np.abs(ru.angle_between(rej2, n2))) > sa)):
    return (False, c, a, r);
  
  return (True, c, a, r);


def fit_cone(p1, n1, p2, n2, p3, n3, eps=0.005, alpha=np.deg2rad(10.0)):
  ''' 
    fit a cone to the three point/normal pairs 
    eps: distance thres for determining if the fit is a valid cylinder
          both points must be < eps from the cylinder
    alpha: angle thres for determining if the fit is a valid cylinder
          both normals must be < alpha from the cylinder normals

    return (valid, apex, axis, opening angle);
      valid is a bool indicating if the fit was valid based on the points and normals
  '''
  p1 = np.asarray(p1);
  n1 = np.asarray(n1);
  p2 = np.asarray(p2);
  n2 = np.asarray(n2);
  p3 = np.asarray(p3);
  n3 = np.asarray(n3);

  # find cone apex (intersection of the 3 planes formed by the point/normal pairs)
  # line normal from plane 1/2 intersection
  (lp, ln) = mu.plane_plane_intersection(p1, n1, p2, n2);
  c = mu.line_plane_intersection(lp, ln, p3, n3);

  # find the axis from the normal of the plane formed by the three points
  pc1 = c + (p1 - c)/np.linalg.norm(p1 - c);
  pc2 = c + (p2 - c)/np.linalg.norm(p2 - c);
  pc3 = c + (p3 - c)/np.linalg.norm(p3 - c);
  a = np.cross(pc2 - pc1, pc3 - pc1);
  a /= np.linalg.norm(a);

  # find opening anlge
  ac = np.array(map(lambda p: ru.angle_between(p - c, a), [p1, p2, p3]));
  w = np.sum(ac)/3.0;

  # check validity

  # project each point onto the axis
  p1_proj_a = np.dot(p1 - c, a) * a;
  p2_proj_a = np.dot(p2 - c, a) * a;
  p3_proj_a = np.dot(p3 - c, a) * a;
  # and rejections
  p1_rej_a = (p1 - c) - p1_proj_a;
  p2_rej_a = (p2 - c) - p2_proj_a;
  p3_rej_a = (p3 - c) - p3_proj_a;

  # projection mag
  d1 = np.linalg.norm(p1_proj_a);
  d2 = np.linalg.norm(p2_proj_a);
  d3 = np.linalg.norm(p3_proj_a);
  # mag of vector from axis to cone edge
  r1 = d1 * np.tan(w);
  r2 = d2 * np.tan(w);
  r3 = d3 * np.tan(w);

  # scale rejections to find the cone point 
  c1 = c + p1_proj_a + (r1/np.linalg.norm(p1_rej_a))*p1_rej_a;
  c2 = c + p2_proj_a + (r2/np.linalg.norm(p2_rej_a))*p2_rej_a;
  c3 = c + p3_proj_a + (r3/np.linalg.norm(p3_rej_a))*p3_rej_a;
  

  # is the point within distance thres?
  distToCone = np.array([np.linalg.norm(p1 - c1), np.linalg.norm(p2 - c2), np.linalg.norm(p3 - c3)]);
  if np.any(distToCone > eps):
    return (False, c, a, w);

  # compute cone normals
  cn1 = np.cross(np.cross(a, (c1 - c)), (c1 - c));
  cn2 = np.cross(np.cross(a, (c2 - c)), (c2 - c));
  cn3 = np.cross(np.cross(a, (c3 - c)), (c3 - c));
  cn1 /= np.linalg.norm(cn1);
  cn2 /= np.linalg.norm(cn2);
  cn3 /= np.linalg.norm(cn3);

  # are the normals close?
  sa = np.sin(alpha);
  if ((np.sin(np.abs(ru.angle_between(cn1, n1))) > sa)
      or (np.sin(np.abs(ru.angle_between(cn2, n2))) > sa)
      or (np.sin(np.abs(ru.angle_between(cn3, n3))) > sa)):
    return (False, c, a, w);

  return (True, c, a, w);



def rand_cylinder(pts, normals):
  ''' fits cylinder to two random points and display the result '''
  rnd1 = random.randint(0, pts.shape[0]-1);
  rnd2 = random.randint(0, pts.shape[0]-1);
  p1 = pts[rnd1, :];
  p2 = pts[rnd2, :];
  n1 = normals[rnd1, :];
  n2 = normals[rnd2, :];

  # fit cylinder
  (valid, c, a, r) = fit_cylinder(p1, n1, p2, n2, eps=.01, alpha=np.deg2rad(10));

  # project points onto axis to find better center
  c += a*(np.dot(a, p1-c) + np.dot(a, p2-c))/2.0;

  print ('valid: %s' % valid);

  # publish points
  pa = create_normals_pose_array(np.array([p1,p2]), np.array([n1,n2]));
  #Debug.pa0Pub.publish(pa);

  # create marker
  cmarker = Debug.cylinder_marker(c, a, r, length=.1);
  Debug.markerPub.publish(cmarker);

  return (p1, n1, p2, n2, valid, c, a, r);

