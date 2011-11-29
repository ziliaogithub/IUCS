#!/usr/bin/env python  

import os
import sys
import time
import copy
import string
import threading
import numpy as np
import scipy as sp

import roslib; roslib.load_manifest('ros_python_util')
import rospy
import cv
import cv_bridge

import tf
import PyKDL as kdl

import message_filters

import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs

import Transform as tr

init = False;
listener = None;
transformer = tf.TransformerROS(True);

def init_utils():
  ''' 
    initializes all utilities that need to be 
      initialized after init_node is called
    
    *** THIS MUST BE CALLED AFTER rospy.init_node() ***
  '''
  global init
  global listener 

  if not init:
  
    rospy.loginfo(__name__ + ': registering tf listener...');
    listener = tf.TransformListener();

    init = True;


def service_proxy(name, msgType, wait=True, timeout=5.0):
  ''' setup ros service '''
  try:
    if (wait):
      rospy.logdebug(__name__ + ': waiting for %s...' % name);
      rospy.wait_for_service(name, timeout);
    rospy.logdebug(__name__ + ': creating service proxy to %s...' % name);
    srvProxy = rospy.ServiceProxy(name, msgType)
    if (wait):
      rospy.logdebug(__name__ + ': %s connection made' % name)
  except rospy.ROSException as err:
    rospy.logfatal(__name__ + ': service connection failed: %s :: %s' % (name, err));
    sys.exit();
  return srvProxy


def wait_for_sync_message(topics, msgTypes, timeout=2.0):
  '''
    similar to rospy.wait_for_message but for time synced messages
  '''
  # TODO: is using globals the best way to do this?
  global data, receivedData
  data = [];
  receivedData = False;

  def sync_callback(*args):
    ''' callback for synced messages '''
    global data, receivedData
    # only store the first message received
    if not receivedData:
      data = args;
      receivedData = True;

  # create syncronizer
  subs = [message_filters.Subscriber(topic, msgType) for (topic, msgType) in zip(topics, msgTypes)];
  synchronizer = message_filters.TimeSynchronizer(subs, 1);
  synchronizer.registerCallback(sync_callback);

  # wait for the data
  t0 = rospy.Time().now().to_sec();
  while (not receivedData and rospy.Time().now().to_sec() - t0 < timeout):
    rospy.sleep(0.01);

  # unregister topics
  [sub.sub.unregister() for sub in subs];

  if not receivedData:
    rospy.logerr(__name__ + ': wait_for_sync_message: no data received for topics %s' % topics);

  return data;


class GenericSubcriber():
  ''' 
    generic subscriber class

    useful for interfactive debugging to quickly create a subscriber
    just keeps a copy of the most recent published data
  '''
  def __init__(self, topic, msgType, nqueue=1):
    rospy.Subscriber(topic, msgType, self.callback, None, nqueue);
    self.data = msgType();

  def callback(self, data):
    self.data = data;

  def get_data(self):
    return self.data;


class ROSBag():
  ''' 
    programatic interface to start and stop rosbag
  '''
  def __init__(self, topics, prefix='', timeout=-1):
    '''
      topics: list or string containing the topics to record
      prefix: string prefix to append to the beginning of each bag
      timeout: optional argument, if its positive then the bag will be started 
                  immediately and record for timeout time. the bag will stop recording
                  after timeout seconds. (It will block during this time.
    '''
    if (type(topics) == str):
      args = topics;
    else:
      args = string.join(topics);
    if (not prefix):
      args += ('-o %s', prefix);

    self.cmd = ('rosbag record %s' % args);

    if (timeout > 0):
      # start and stop log after timeout
      self.start();
      self.stop(timeout);

  def start(self):
    ''' start recording '''
    self.process = subprocess.Popen(self.cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE);
    t0 = rospy.Time().now();

  def get_cids(self):
    ''' return child ids of the bag process '''
    # find child process ids
    pgrepProcess = subprocess.Popen('pgrep -P %d' % self.process.pid, shell=True, stdout=subprocess.PIPE);
    pgrepProcess.wait();
    return map(lambda cid: int(string.strip(cid)), pgrepProcess.stdout.readlines());

  def stop(self, timeout=-1, timeoutFromStart=False):
    ''' stops the bag process '''
    if (timeout > 0):
      if (timeoutFromStart):
        t0 = self.t0;
      else:
        t0 = rospy.Time().now();

      # wait timeout
      rospy.sleep(timeout - (rospy.Time().now().to_sec() - t0.to_sec()));

    # send interrupts to the bagging process
    cids = self.get_cids();
    map(lambda cid: os.kill(cid, signal.SIGINT), cids);
    # wait for the bagging process to finish
    self.process.wait();



colors = {};
colors['black'] = colors['k'] = np.array([0, 0, 0], np.uint8);
colors['white'] = colors['w'] = np.array([255, 255, 240], np.uint8);
colors['red'] = colors['r'] = np.array([255, 0, 0], np.uint8);
colors['green'] = colors['g'] = np.array([0, 255, 0], np.uint8);
colors['blue'] = colors['b'] = np.array([0, 0, 255], np.uint8);
colors['yellow'] = colors['y'] = np.array([255, 255, 0], np.uint8);
colors['magenta'] = colors['m'] = np.array([255, 0, 255], np.uint8);
colors['cyan'] = colors['c'] = np.array([0, 255, 255], np.uint8);


def color2rgb(c):
  ''' returns r,g,b values (0.0 - 1.0) for the given color '''
  global colors

  if (hasattr(c, '__iter__') and len(c) >= 3):
    # assumed to be an iterable already in r,g,b format
    return np.array(int(c[:3]));
  elif type(c) == str:
    # get the color from the dictionary
    c = string.lower(c);
    if c in colors.keys():
      return colors[c].copy();

  rospy.logwarn(rospy.get_name() + ':' + __name__ + ': unkown color %s' % str(c));
  return colors['k'].copy();



def to_Vector3(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/Vector3 message
    
    supported input:
      - ROS Pose message (orientation ignored)
      - ROS PoseStamped message (orientation ignored)
      - ROS Point message
      - ROS PointStamped message
      - ROS Point32 message
      - ROS Transform message (orientation ignored)
      - ROS TransformStamped message (orientation ignored)
      - ROS Vector3 message
      - ROS Vector3Stamped message
      - kdl Frame (orientation ignored)
      - kdl Vector
      - tf transform (orientation ignored)
      - iterable of length 3 (list, tuple, np.array, ...)
      - individual x,y,z values
      - named x, y, z arguments.  (minimum of 1 can be specified)
      - no arguments will result in the zero point

    keyword args:
      - x, y, z

    Note: if more than 1 argument then they should be in the order they appear
  '''

  # None passed in
  if (len(args) == 1 and args[0] == None):
    return geometry_msgs.Vector3(0.0, 0.0, 0.0);
  
  if (len(args) == 1):
    # Pose
    if (type(args[0]) == geometry_msgs.Pose):
      position = args[0].position;
      return geometry_msgs.Vector3(position.x, position.y, position.z);
    # PoseStamped
    if (type(args[0]) == geometry_msgs.PoseStamped):
      position = args[0].pose.position;
      return geometry_msgs.Vector3(position.x, position.y, position.z);
    # Point
    if (type(args[0]) == geometry_msgs.Point):
      return geometry_msgs.Vector3(args[0].x, args[0].y, args[0].z);
    # PointStamed
    if (type(args[0]) == geometry_msgs.PointStamped):
      return geometry_msgs.Vector3(args[0].point.x, args[0].point.y, args[0].point.z);
    # Point32
    if (type(args[0]) == geometry_msgs.Point32):
      return geometry_msgs.Vector3(args[0].x, args[0].y, args[0].z);
    # Transform
    if (type(args[0]) == geometry_msgs.Transform):
      return geometry_msgs.Vector3(args[0].translation.x, 
                                   args[0].translation.y,
                                   args[0].translation.z);
    # TransformStamped
    if (type(args[0]) == geometry_msgs.TransformStamped):
      return geometry_msgs.Vector3(args[0].transform.translation.x, 
                                   args[0].transform.translation.y,
                                   args[0].transform.translation.z);
    # Vector3
    if (type(args[0]) == geometry_msgs.Vector3):
      return copy.deepcopy(args[0]);
    # Vector3Stamped
    if (type(args[0]) == geometry_msgs.Vector3Stamped):
      return copy.deepcopy(args[0].vector);
    # kdl Frame
    if (type(args[0]) == kdl.Frame):
      return geometry_msgs.Vector3(args[0].p.x(), args[0].p.y(), args[0].p.z());
    # kdl Vector
    if (type(args[0]) == kdl.Vector):
      return geometry_msgs.Vector3(args[0].x(), args[0].y(), args[0].z());
    # 4x4 numpy array
    if (type(args[0]) == np.ndarray and args[0].shape == (4,4)):
      return geometry_msgs.Vector3(args[0][0,3], args[0][1,3], args[0][2,3]);
    # tf transform ((x,y,z), (x,y,z,w))
    if (hasattr(args[0], '__iter__') and len(args[0]) == 2 and len(args[0][0]) == 3):
      return geometry_msgs.Vector3(*args[0][0][:3]); 
    # iterable length 3
    if (hasattr(args[0], '__iter__') and len(args[0]) >= 3):
      return geometry_msgs.Vector3(*args[0][:3]);

  # 3 arguments
  elif (len(args) == 3):
    return geometry_msgs.Vector3(*args);
  
  # x, y, or z keyword arguments
  elif ('x' in kwargs or 'y' in kwargs or 'z' in kwargs):
    x = 0.0;
    y = 0.0;
    z = 0.0;
    if ('x' in kwargs):
      x = kwargs['x'];
    if ('y' in kwargs):
      y = kwargs['y'];
    if ('z' in kwargs):
      z = kwargs['z'];
    return geometry_msgs.Vector3(x, y, z);

  # no arguments 
  elif (len(args) == 0):
    return geometry_msgs.Vector3(0.0, 0.0, 0.0);
  

def to_Vector3Stamped(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/Vector3Stamped message

    supported input:
      - ROS Pose message (orientation ignored)
      - ROS PoseStamped message (orientation ignored)
      - ROS Point message
      - ROS PointStamped message
      - ROS Point32 message
      - ROS Transform message (orientation ignored)
      - ROS TransformStamped message (orientation ignored)
      - ROS Vector3 message
      - ROS Vector3Stamped message
      - kdl Frame (orientation ignored)
      - kdl Vector
      - tf transform (orientation ignored)
      - iterable of length 3 (list, tuple, np.array, ...)
      - individual x,y,z values
      - named x, y, z arguments.  (minimum of 1 can be specified)
      - no arguments will result in the zero point

    keyword args:
      - frame_id 
      - stamp
      - x, y, z

    'frame_id' and 'stamp' logic:
     1. If frame_id or stamp are specified as keywords, they will be used, 
        otherwise,
     2. if the passed in argument is a *Stamped type, the frame_id and stamp
        will be taked from that, otherwise,
     3. the rospy.BASE_FRAME and 0 time stamp will be used.
    
    Note: if more than 1 argument then they should be in the order they appear
  '''
  
  # get point
  v3s = geometry_msgs.Vector3Stamped();
  v3s.vector = to_Vector3(*args, **kwargs);
  
  # set frame id
  if ('frame_id' in kwargs.keys()):
    v3s.header.frame_id = kwargs['frame_id'];
  elif ( (len(args) == 1 and args[0] != None) and 
         (type(args[0]) == geometry_msgs.PoseStamped or
          type(args[0]) == geometry_msgs.PointStamped or
          type(args[0]) == geometry_msgs.TransformStamped or 
          type(args[0]) == geometry_msgs.Vector3Stamped) ):
    v3s.header.frame_id = args[0].header.frame_id
  
  # set stamp
  if 'stamp' in kwargs.keys():
    v3s.header.stamp = kwargs['stamp'];
  # else, use existing stamp if passed in a *Stamped message
  elif ( (len(args) == 1 and args[0] != None) and 
         (type(args[0]) == geometry_msgs.PoseStamped or
          type(args[0]) == geometry_msgs.PointStamped or
          type(args[0]) == geometry_msgs.TransformStamped or 
          type(args[0]) == geometry_msgs.Vector3Stamped) ):
    v3s.header.stamp = args[0].header.stamp 
  
  return v3s;

  
def to_Point(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/Point message
    
    supported input:
      - ROS Pose message (orientation ignored)
      - ROS PoseStamped message (orientation ignored)
      - ROS Point message
      - ROS PointStamped message
      - ROS Point32 message
      - ROS Transform message (orientation ignored)
      - ROS TransformStamped message (orientation ignored)
      - ROS Vector3 message
      - ROS Vector3Stamped message
      - kdl Frame (orientation ignored)
      - kdl Vector
      - tf transform (orientation ignored)
      - iterable of length 3 (list, tuple, np.array, ...)
      - individual x,y,z values
      - named x, y, z arguments.  (minimum of 1 can be specified)
      - no arguments will result in the zero point

    keyword args:
      - x, y, z

    Note: if more than 1 argument then they should be in the order they appear
  '''

  # None passed in
  if (len(args) == 1 and args[0] == None):
    return geometry_msgs.Point(0.0, 0.0, 0.0);
  
  if (len(args) == 1):
    # Pose
    if (type(args[0]) == geometry_msgs.Pose):
      return copy.deepcopy(args[0].position);
    # PoseStamped
    if (type(args[0]) == geometry_msgs.PoseStamped):
      return copy.deepcopy(args[0].pose.position);
    # Point
    if (type(args[0]) == geometry_msgs.Point):
      return copy.deepcopy(args[0]);
    # PointStamed
    if (type(args[0]) == geometry_msgs.PointStamped):
      return copy.deepcopy(args[0].point);
    # Point32
    if (type(args[0]) == geometry_msgs.Point32):
      return geometry_msgs.Point(args[0].x, args[0].y, args[0].z);
    # Transform
    if (type(args[0]) == geometry_msgs.Transform):
      return geometry_msgs.Point(args[0].translation.x, 
                                 args[0].translation.y,
                                 args[0].translation.z);
    # TransformStamped
    if (type(args[0]) == geometry_msgs.TransformStamped):
      return geometry_msgs.Point(args[0].transform.translation.x, 
                                 args[0].transform.translation.y,
                                 args[0].transform.translation.z);
    # Vector3
    if (type(args[0]) == geometry_msgs.Vector3):
      return geometry_msgs.Point(args[0].x, args[0].y, args[0].z);
    # Vector3Stamped
    if (type(args[0]) == geometry_msgs.Vector3Stamped):
      return geometry_msgs.Point(args[0].vector.x, args[0].vector.y, args[0].vector.z);
    # kdl Frame
    if (type(args[0]) == kdl.Frame):
      return geometry_msgs.Point(args[0].p.x(), args[0].p.y(), args[0].p.z());
    # kdl Vector
    if (type(args[0]) == kdl.Vector):
      return geometry_msgs.Point(args[0].x(), args[0].y(), args[0].z());
    # 4x4 numpy array
    if (type(args[0]) == np.ndarray and args[0].shape == (4,4)):
      return geometry_msgs.Point(args[0][0,3], args[0][1,3], args[0][2,3]);
    # tf transform ((x,y,z), (x,y,z,w))
    if (hasattr(args[0], '__iter__') and len(args[0]) == 2 and len(args[0][0]) == 3):
      return geometry_msgs.Point(*args[0][0][:3]); 
    # iterable length 3
    if (hasattr(args[0], '__iter__') and len(args[0]) >= 3):
      return geometry_msgs.Point(*args[0][:3]);

  elif (len(args) == 3):
    return geometry_msgs.Point(*args);
  
  elif ('x' in kwargs or 'y' in kwargs or 'z' in kwargs):
    x = 0.0;
    y = 0.0;
    z = 0.0;
    if ('x' in kwargs):
      x = kwargs['x'];
    if ('y' in kwargs):
      y = kwargs['y'];
    if ('z' in kwargs):
      z = kwargs['z'];
    return geometry_msgs.Point(x, y, z);

  elif (len(args) == 0):
    return geometry_msgs.Point(0.0, 0.0, 0.0);
  

def to_Point32(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/Point32 message
    
    supported input:
      - ROS Pose message (orientation ignored)
      - ROS PoseStamped message (orientation ignored)
      - ROS Point message
      - ROS PointStamped message
      - ROS Point32 message
      - ROS Transform message (orientation ignored)
      - ROS TransformStamped message (orientation ignored)
      - ROS Vector3 message
      - ROS Vector3Stamped message
      - kdl Frame (orientation ignored)
      - kdl Vector
      - tf transform (orientation ignored)
      - iterable of length 3 (list, tuple, np.array, ...)
      - individual x,y,z values
      - named x, y, z arguments.  (minimum of 1 can be specified)
      - no arguments will result in the zero point

    keyword args:
      - x, y, z

    Note: if more than 1 argument then they should be in the order they appear
  '''
  # get Point
  point = to_Point(*args, **kwargs)
  
  return geometry_msgs.Point32(point.x, point.y, point.z)


def to_PointStamped(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/PointStamped message

    supported input:
      - ROS Pose message (orientation ignored)
      - ROS PoseStamped message (orientation ignored)
      - ROS Point message
      - ROS PointStamped message
      - ROS Point32 message
      - ROS Transform message (orientation ignored)
      - ROS TransformStamped message (orientation ignored)
      - ROS Vector3 message
      - ROS Vector3Stamped message
      - kdl Frame (orientation ignored)
      - kdl Vector
      - tf transform (orientation ignored)
      - iterable of length 3 (list, tuple, np.array, ...)
      - individual x,y,z values
      - named x, y, z arguments.  (minimum of 1 can be specified)
      - no arguments will result in the zero point

    keyword args:
      - frame_id 
      - stamp
      - x, y, z

    'frame_id' and 'stamp' logic:
     1. If frame_id or stamp are specified as keywords, they will be used, 
        otherwise,
     2. if the passed in argument is a *Stamped type, the frame_id and stamp
        will be taked from that, otherwise,
     3. the rospy.BASE_FRAME and 0 time stamp will be used.
    
    Note: if more than 1 argument then they should be in the order they appear
  '''
  
  # get point
  pts = geometry_msgs.PointStamped();
  pts.point = to_Point(*args, **kwargs);
  
  if ('frame_id' in kwargs.keys()):
    pts.header.frame_id = kwargs['frame_id'];
  elif ( (len(args) == 1 and args[0] != None) and 
         (type(args[0]) == geometry_msgs.PoseStamped or
          type(args[0]) == geometry_msgs.PointStamped or
          type(args[0]) == geometry_msgs.TransformStamped or 
          type(args[0]) == geometry_msgs.Vector3Stamped) ):
    pts.header.frame_id = args[0].header.frame_id
  
  if ('stamp' in kwargs.keys()):
    pts.header.stamp = kwargs['stamp'];
  elif ( (len(args) == 1 and args[0] != None) and 
         (type(args[0]) == geometry_msgs.PoseStamped or
          type(args[0]) == geometry_msgs.PointStamped or
          type(args[0]) == geometry_msgs.TransformStamped or 
          type(args[0]) == geometry_msgs.Vector3Stamped) ):
    pts.header.stamp = args[0].header.stamp 
  
  return pts;


def to_Quaternion(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/Quaternion message
    
    supported input:
      - ROS Pose message (position ignored)
      - ROS PoseStamped message (position ignored)
      - ROS Transform message (position ignored)
      - ROS TransformStamped message (position ignored)
      - ROS Quaternion message
      - ROS QuaternionStamped message
      - kdl Frame (position ignored)
      - kdl Rotation
      - tf transform (position ignored)
      - iterable of length 4 (list, tuple, np.array, ...) (x, y, z, w)
      - a 3x3 or 4x4 numpy array
      - individual x, y, z, w values
      - named x, y, z, w arguments.  (minimum of 1 can be specified)
      - no arguments will result in an identity rotation

    keyword args:
      - x, y, z

    Note: if more than 1 argument then they should be in the order they appear
  '''

  # None passed in
  if (len(args) == 1 and args[0] == None):
    return geometry_msgs.Quaternion(0,0,0,1);
  
  if (len(args) == 1):
    # Quaternion
    if (type(args[0]) == geometry_msgs.Quaternion):
      return copy.deepcopy(args[0]);    
    # QuaternionStamped
    if (type(args[0]) == geometry_msgs.QuaternionStamped):
      return copy.deepcopy(args[0].quaternion);
    # Pose
    if (type(args[0]) == geometry_msgs.Pose):
      return copy.deepcopy(args[0].orientation);
    # PoseStamped
    if (type(args[0]) == geometry_msgs.PoseStamped):
      return copy.deepcopy(args[0].pose.orientation);
    # Transform
    if (type(args[0]) == geometry_msgs.Transform):
      return copy.deepcopy(args[0].rotation);
    # TransformStamped
    if (type(args[0]) == geometry_msgs.TransformStamped):
      return copy.deepcopy(args[0].transform.rotation);
    # kdl Frame
    if (type(args[0]) == kdl.Frame):
      return geometry_msgs.Quaternion(*args[0].M.GetQuaternion())
    # kdl Rotation
    if (type(args[0]) == kdl.Rotation):
      return geometry_msgs.Quaternion(*args[0].GetQuaternion())
    
    # 3x3 or 4x4 numpy array
    if (type(args[0]) == np.ndarray and args[0].ndim == 2):
      (r, c) = args[0].shape
      if (r >= 3 and c >= 3):
        rot = kdl.Rotation(args[0][0,0], args[0][0,1], args[0][0,2],
                           args[0][1,0], args[0][1,1], args[0][1,2],
                           args[0][2,0], args[0][2,1], args[0][2,2])
        return geometry_msgs.Quaternion(*rot.GetQuaternion())
      
    # tf transform ((x,y,z), (x,y,z,w))
    if (hasattr(args[0], '__iter__') and len(args[0]) == 2 and len(args[0][1]) == 4):
      return geometry_msgs.Quaternion(*args[0][1][:4]);
    # iterable length 4
    if (hasattr(args[0], '__iter__') and len(args[0]) >= 4):
      return geometry_msgs.Quaternion(*args[0][:4]);

  # 4 arguments
  elif (len(args) == 4):
    return geometry_msgs.Quaternion(*args);
  
  # x, y, z, or w keyword arguments
  elif ('x' in kwargs or 'y' in kwargs or 'z' in kwargs or 'w' in kwargs):
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 0.0;
    if ('x' in kwargs):
      x = kwargs['x'];
    if ('y' in kwargs):
      y = kwargs['y'];
    if ('z' in kwargs):
      z = kwargs['z'];
    if ('w' in kwargs):
      w = kwargs['w'];

    return geometry_msgs.Quaternion(x, y, z, w);

  elif (len(args) == 0):
    return geometry_msgs.Quaternion(0.0, 0.0, 0.0, 1.0);
  


def to_Pose(*args):
  ''' 
    converts the input to a geometry_msgs/Pose message
    
    supported input:
      * single argument:
         - ROS Pose message
         - ROS PoseStamped message
         - ROS Transform message
         - ROS TransformStamped message
         - kdl frame
         - tf transform
         - numpy 4x4 transform matrix
         - anything to_Point() takes. (Will create a Pose with the specified 
           point and an identity rotation.  May generate warnings.)
         - anything to_Quaternion() takes. (Will create a Pose with a zero
           point and the specified rotation.  May generate warnings.)
      * two arguments:
         - first argument: anything to_Point() takes
         - second argument: anything to_Quaternion() takes
      * no arguments will result in the identity pose

    Note: if more than 1 argument then they should be in the order they appear
    Note: array form of quaternion is assumed to be (x, y, z, w)
  '''
  if (len(args) == 0):
    return geometry_msgs.Pose(geometry_msgs.Point(0.0, 0.0, 0.0), geometry_msgs.Quaternion(0.0, 0.0, 0.0, 1.0));

  elif (len(args) == 1):
    point = to_Point(args[0]);
    quat = to_Quaternion(args[0]);

    # return pose
    return geometry_msgs.Pose(point, quat)

  elif (len(args) == 2):
    return geometry_msgs.Pose(to_Point(args[0]), to_Quaternion(args[1]));
  

def to_PoseStamped(*args, **kwargs):
  ''' 
    converts the input to a geometry_msgs/PoseStamped message

    supported input:
      * single argument:
         - ROS Pose message
         - ROS PoseStamped message
         - ROS Transform message
         - ROS TransformStamped message
         - kdl frame
         - tf transform
         - numpy 4x4 transform matrix
         - anything to_Point() takes. (Will create a Pose with the specified 
           point and an identity rotation.  May generate warnings.)
         - anything to_Quaternion() takes. (Will create a Pose with a zero
           point and the specified rotation.  May generate warnings.)
      * two arguments:
         - first argument: anything to_Point() takes
         - second argument: anything to_Quaternion() takes
      * no arguments will result in the identity pose

    keyword args:
      - frame_id 
      - stamp

    'frame_id' and 'stamp' logic:
     1. If frame_id or stamp are specified as keywords, they will be used, 
        otherwise,
     2. if the first passed in argument is a *Stamped type, the frame_id and stamp
        will be taked from that, otherwise,
     3. if the second passed in argument is a *Stamped type, the frame_id and stamp
        will be taked from that, otherwise,
     4. the rospy.BASE_FRAME and 0 time stamp will be used.

    Note: if more than 1 argument then they should be in the order they appear
    Note: array form of quaternion is assumed to be (x, y, z, w)
  '''

  # get pose
  ps = geometry_msgs.PoseStamped();
  ps.pose = to_Pose(*args);

  if ('frame_id' in kwargs.keys()):
    ps.header.frame_id = kwargs['frame_id'];
  elif ( (len(args) >= 1 and args[0] != None) and 
         (type(args[0]) == geometry_msgs.PoseStamped or
          type(args[0]) == geometry_msgs.PointStamped or
          type(args[0]) == geometry_msgs.QuaternionStamped or
          type(args[0]) == geometry_msgs.TransformStamped or 
          type(args[0]) == geometry_msgs.Vector3Stamped) ):
    ps.header.frame_id = args[0].header.frame_id
  elif ( (len(args) >= 2 and args[1] != None) and 
         (type(args[1]) == geometry_msgs.PoseStamped or
          type(args[1]) == geometry_msgs.QuaternionStamped or
          type(args[1]) == geometry_msgs.TransformStamped) ):
    ps.header.frame_id = args[1].header.frame_id
  
  if ('stamp' in kwargs.keys()):
    ps.header.stamp = kwargs['stamp'];
  elif ( (len(args) >= 1 and args[0] != None) and 
         (type(args[0]) == geometry_msgs.PoseStamped or
          type(args[0]) == geometry_msgs.PointStamped or
          type(args[0]) == geometry_msgs.QuaternionStamped or
          type(args[0]) == geometry_msgs.TransformStamped or 
          type(args[0]) == geometry_msgs.Vector3Stamped) ):
    ps.header.stamp = args[0].header.stamp 
  # else, use existing stamp if passed in a *Stamped message as argument 2
  elif ( (len(args) >= 2 and args[1] != None) and 
         (type(args[1]) == geometry_msgs.PoseStamped or
          type(args[1]) == geometry_msgs.QuaternionStamped or
          type(args[1]) == geometry_msgs.TransformStamped) ):
    ps.header.stamp = args[1].header.stamp

  return ps;


def to_PointCloud(*args, **kwargs):
  ''' 
    converts the input to a sensor_msgs/PointCloud message

    supported input:
      - ROS PointCloud messages
      - iterable of ROS Point messages
      - iterable of kdl Vectors
      - iterable of iterables
      - numpy array Nx3 or 3xN (if N == 3 then it is assumed Nx3)
      - no arguments will result in an empty PointCloud

    keyword args:
      - frame_id 
      - stamp

    NOTE: mixing types is not supported
  '''
  if (len(args) > 0 and type(args[0]) == sensor_msgs.PointCloud):
    return copy.deepcopy(args[0]);
  
  pc = sensor_msgs.PointCloud();
  if (len(args) > 0):
    pts = args[0];
    # numpy array
    if (type(pts) == np.ndarray):
      # 3xN (N > 3)
      if (max(pts.shape) > 3 and pts.shape[1] > pts.shape[0]):
        pts = pts.T;
      # Nx3 or 3xN (N < 3)
      elif (pts.shape != (3,3) and pts.shape[0] == 3):
        pts = pts.T;
      pc.points = [geometry_msgs.Point32(p[0], p[1], p[2]) for p in pts];

    elif (hasattr(pts, '__iter__') and len(args[0]) > 0):
      # Point32
      if (type(pts[0]) == geometry_msgs.Point32):
        pc.points = copy.deepcopy(pts); 
      # Point
      elif (type(pts[0]) == geometry_msgs.Point):
        pc.points = [geometry_msgs.Point32(p.x, p.y, p.z) for p in pts];
      # kdl Vector
      elif (type(pts[0]) == kdl.Vector):
        pc.points = [geometry_msgs.Point32(p.x(), p.y(), p.z()) for p in pts];
      # iterable
      elif hasattr(pts[0], '__iter__'):
        pc.points = [geometry_msgs.Point32(p[0], p[1], p[2]) for p in pts];

  if ('frame_id' in kwargs.keys()):
    pc.header.frame_id = kwargs['frame_id'];

  if ('stamp' in kwargs.keys()):
    pc.header.stamp = kwargs['stamp'];

  return pc; 


def header(stamp=rospy.Time(), frame_id=''):
  ''' creates a ros header '''
  return rospy.Header(seq=0, stamp=stamp, frame_id=frame_id);


def to_array(*args):
  ''' 
    converts the input to numpy array format

    supported input:
      - ROS PointCloud message
      - ROS Pose/Stamped message
      - ROS Quaternion/Stamped message
      - ROS Point/Stamped message
      - ROS Vector3/Stamped message
      - ROS Image message (returned in RGB format)
      - kdl frame
      - kdl Rotation
      - kdl Vector
      - iterable 

    NOTE: point clouds are returned as Nx3 arrays
    NOTE: Poses and Rotations will be returned in 4x4 matrix format 
  '''
  # numpy array
  if (len(args) > 0 and type(args[0]) == np.ndarray):
    return args[0].copy();

  # PointCloud
  elif (type(args[0]) == sensor_msgs.PointCloud):
    return np.array([(p.x, p.y, p.z) for p in args[0].points]);
  # PoseStamped
  elif (type(args[0]) == geometry_msgs.PoseStamped):
    return tr.rospose2tr(args[0].pose);
  # Pose
  elif (type(args[0]) == geometry_msgs.Pose):
    return tr.rospose2tr(args[0]);
  # QuaternionStamped
  elif (type(args[0]) == geometry_msgs.QuaternionStamped):
    return tr.rosq2rot(args[0].quaternion);
  # Quaternion
  elif (type(args[0]) == geometry_msgs.Quaternion):
    return tr.rosq2rot(args[0]);
  # Vector3Stamped
  elif (type(args[0]) == geometry_msgs.Vector3Stamped):
    return np.array([args[0].vector.x, args[0].vector.y, args[0].vector.z]);
  # Vector3
  elif (type(args[0]) == geometry_msgs.Vector3):
    return np.array([args[0].x, args[0].y, args[0].z]);
  # PointStamped
  elif (type(args[0]) == geometry_msgs.PointStamped):
    return np.array([args[0].point.x, args[0].point.y, args[0].point.z]);
  # Point
  elif (type(args[0]) == geometry_msgs.Point
        or type(args[0]) == geometry_msgs.Point32):
    return np.array([args[0].x, args[0].y, args[0].z]);
  # ROS Image
  elif (type(args[0]) == sensor_msgs.Image):
    bridge = cv_bridge.CvBridge();
    cvimg = bridge.imgmsg_to_cv(args[0]);
    return np.asarray(cvimg);
  # kdl Frame
  elif (type(args[0]) == kdl.Frame):
    return tr.kdlf2tr(args[0]);
  # kdl Rotation
  elif (type(args[0]) == kdl.Rotation):
    return tr.kdlr2tr(args[0]);
  # kdl Vector
  elif (type(args[0]) == kdl.Vector):
    return np.array([args[0].x(), args[0].y(), args[0].z()]);
  # iterable
  elif hasattr(args[0], '__iter__'):
    if (len(args[0]) > 0 
          and type(args[0][0]) == geometry_msgs.Point or type(args[0][0]) == geometry_msgs.Point32):
      return np.array([(p.x, p.y, p.z) for p in args[0]]);
    else:
      return np.array(args[0]);


def to_KDLFrame(*args):
  p = to_Pose(*args);
  
  kdlR = kdl.Rotation.Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  kdlT = kdl.Vector(p.position.x, p.position.y, p.position.z);

  return kdl.Frame(kdlR, kdlT);


def to_KDLVector(*args):
  v = to_Vector3(*args);

  return kdl.Vector(v.x, v.y, v.z);


def to_KDLRotation(*args):
  r = to_Quaternion(*args);

  return kdl.Rotation.Quaternion(r.x, r.y, r.z, r.w);


##########################################


def transformPoint(target_frame, ps, waitTimeout=0.1):
  '''
    wrapper around tf.listener.transformPoint

    If the transform fails due to timing the most current transform is used
  '''
  global listener
  try:
    listener.waitForTransform(target_frame, ps.header.frame_id, ps.header.stamp, rospy.Duration(waitTimeout))
    return listener.transformPoint(target_frame, ps); 
  except (tf.Exception, tf.ExtrapolationException), ex:
    # transform failed, use most current transform
    rospy.logwarn(__name__ + ': unable to transform point: %s' % ex);
    rospy.logwarn(__name__ + ': proceeding with most recent transform');
    ps = copy.deepcopy(ps);
    ps.header.stamp = rospy.Time();
    return listener.transformPoint(target_frame, ps);


def transformVector3(target_frame, v3s, waitTimeout=0.1):
  '''
    wrapper around tf.listener.transformVector3

    If the transform fails due to timing the most current transform is used
  '''
  global listener
  try:
    listener.waitForTransform(target_frame, v3s.header.frame_id, v3s.header.stamp, rospy.Duration(waitTimeout))
    return listener.transformVector3(target_frame, v3s); 
  except (tf.Exception, tf.ExtrapolationException), ex:
    # transform failed, use most current transform
    rospy.logwarn(__name__ + ': unable to transform vector3: %s' % ex);
    rospy.logwarn(__name__ + ': proceeding with most recent transform');
    v3s = copy.deepcopy(v3s);
    v3s.header.stamp = rospy.Time();
    return listener.transformVector3(target_frame, v3s);


def transformQuaternion(target_frame, qs, waitTimeout=0.1):
  '''
    wrapper around tf.listener.transformQuaternion

    If the transform fails due to timing the most current transform is used
  '''
  global listener
  try:
    listener.waitForTransform(target_frame, qs.header.frame_id, qs.header.stamp, rospy.Duration(waitTimeout))
    return listener.transformQuaternion(target_frame, qs); 
  except (tf.Exception, tf.ExtrapolationException), ex:
    # transform failed, use most current transform
    rospy.logwarn(__name__ + ': unable to transform quaternion: %s' % ex);
    rospy.logwarn(__name__ + ': proceeding with most recent transform');
    qs = copy.deepcopy(qs);
    qs.header.stamp = rospy.Time();
    return listener.transformQuaternion(target_frame, qs);


def transformPose(target_frame, ps, waitTimeout=0.1):
  '''
    wrapper around tf.listener.transformPose

    If the transform fails due to timing the most current transform is used
  '''
  global listener
  try:
    listener.waitForTransform(target_frame, ps.header.frame_id, ps.header.stamp, rospy.Duration(waitTimeout))
    return listener.transformPose(target_frame, ps); 
  except (tf.Exception, tf.ExtrapolationException), ex:
    # transform failed, use most current transform
    rospy.logwarn(__name__ + ': unable to transform pose: %s' % ex);
    rospy.logwarn(__name__ + ': proceeding with most recent transform');
    ps = copy.deepcopy(ps);
    ps.header.stamp = rospy.Time();
    return listener.transformPose(target_frame, ps);


def transformPointCloud(target_frame, point_cloud, waitTimeout=0.1):
  '''
    wrapper around tf.listener.transformPointCloud

    If the transform fails due to timing the most current transform is used
  '''
  global listener
  try:
    listener.waitForTransform(target_frame, point_cloud.header.frame_id, point_cloud.header.stamp, rospy.Duration(waitTimeout))
    return listener.transformPointCloud(target_frame, point_cloud); 
  except (tf.Exception, tf.ExtrapolationException), ex:
    # transform failed, use most current transform
    rospy.logwarn(__name__ + ': unable to transform point cloud: %s' % ex);
    rospy.logwarn(__name__ + ': proceeding with most recent transform');
    point_cloud = copy.deepcopy(point_cloud);
    point_cloud.header.stamp = rospy.Time();
    return listener.transformPointCloud(target_frame, point_cloud);


def lookupTransform(dstFrame, srcFrame, stamp=rospy.Time(), waitTimeout=0.1):
  ''' returns the 4x4 transform for the given frames '''
  global listener
  try:
    listener.waitForTransform(dstFrame, srcFrame, stamp, rospy.Duration(waitTimeout))
    (t,q) = listener.lookupTransform(dstFrame, srcFrame, stamp);
  except (tf.Exception, tf.ExtrapolationException), ex:
    # transform failed, use most current transform
    rospy.logwarn(__name__ + ': unable to get transform : %s' % ex);
    rospy.logwarn(__name__ + ': proceeding with most recent transform');
    (t,q) = listener.lookupTransform(dstFrame, srcFrame, rospy.Time());

  return np.dot(tr.trans(t), tr.rosq2rot(q));


def get_frame_id(msg):
  ''' 
    returns the frame id for the input
    '' if no frame is specified
  '''
  if hasattr(msg, 'header'):
    return msg.header.frame_id;
  elif hasattr(msg, 'frame_id'):
    return obj.frame_id;
  else:
    return '';


