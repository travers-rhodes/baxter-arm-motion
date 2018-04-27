#!/usr/bin/python
import rospy
import numpy as np

from baxter_arm_motion.msg import Tracking
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

####
## This method generateMessage can be modified to fill in a real trajectory.
## Right now this just publishes a circle for the robot arm to follow
####
# given a number of seconds since the starting time
# return a tracking object that has
# * the current point location and
# * the future predicted point location 1 second from now  
def generateMessage(secondsSinceStart):
  msg = Tracking()
  current = circle_path(secondsSinceStart)
  future = circle_path(secondsSinceStart + 1)
  msg.current = Point(current[0], current[1], current[2])
  msg.future = Point(future[0], future[1], future[2])
  msg.curTime = rospy.get_rostime()
  return msg

def generateFullTrajMsg():
  mesg = Marker()
  mesg.type = Marker.LINE_STRIP
  mesg.id = 0;
  mesg.scale.x = 0.05 # = Vector3(1.0, 0, 0)
  mesg.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
  mesg.points = [circle_path_point(t) for t in np.linspace(0,np.pi * 8.01)]
  mesg.header.frame_id = 'torso'
  return mesg 

def generateTargetVisMsg(secondsSinceStart):
  mesg = Marker()
  mesg.type = Marker.POINTS
  mesg.id = 0;
  mesg.scale.x = 0.05 # = Vector3(1.0, 0, 0)
  mesg.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
  mesg.points = [circle_path_point(secondsSinceStart)]
  mesg.header.frame_id = 'torso'
  return mesg 
   
def circle_path_point(secondsSinceStart):
  vec = circle_path(secondsSinceStart)
  return Point(vec[0], vec[1], vec[2])

def circle_path(secondsSinceStart):
  center = np.array([ 0.772040974663,
                     -0.724013073822,
                      0.0569658745728])

  omg = 0.25
  radius = 0.1
  point = center + radius * np.array([0,np.sin(omg * secondsSinceStart), np.cos(omg * secondsSinceStart)])
  return point

class TargetGeneration:
  def __init__(self):
    # publish at 1 Hz
    self.freq = 1
    self.publisher = rospy.Publisher("/follow/target_point", Tracking, queue_size = 10)
    self.trajpublisher = rospy.Publisher("/follow/full_traj", Marker, queue_size = 10)
    self.targpublisher= rospy.Publisher("/follow/target_mark", Marker, queue_size = 10)
    self.truthpublisher= rospy.Publisher("/follow/truth", Marker, queue_size = 10)
    rate = rospy.Rate(self.freq)
    self.starttime = rospy.get_rostime()
    while self.starttime.secs == 0:
      self.starttime = rospy.get_rostime()
    print self.starttime
    print("sending targeting messages")
    rospy.Timer(rospy.Duration(1.0/self.freq), self.slowCallback)
    rospy.Timer(rospy.Duration(1.0/200), self.fastCallback)

  def slowCallback(self,event):
      msg = generateMessage(rospy.get_rostime().to_sec() - self.starttime.to_sec())
      self.sendMessage(msg)
      pathmsg = generateFullTrajMsg()
      self.trajpublisher.publish(pathmsg)
      targmsg = generateTargetVisMsg(rospy.get_rostime().to_sec() - self.starttime.to_sec())
      self.targpublisher.publish(targmsg)

  def fastCallback(self, event):
      targmsg = generateTargetVisMsg(rospy.get_rostime().to_sec() - self.starttime.to_sec())
      self.truthpublisher.publish(targmsg)
        

  def sendMessage(self, msg):
    self.publisher.publish(msg) 

if __name__=="__main__":
  rospy.init_node("target_generator")
  t = TargetGeneration()
  rospy.spin()
