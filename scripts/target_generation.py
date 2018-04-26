#!/usr/bin/python
import rospy
import numpy as np

from baxter_arm_motion.msg import Tracking
from geometry_msgs.msg import Point

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
  return msg

def circle_path(secondsSinceStart):
  center = np.array([0.1, -0.7, 0.3])
  omg = 0.1
  point = center + 0 * np.array([0,np.sin(omg * secondsSinceStart), np.cos(omg * secondsSinceStart)])
  return point

class TargetGeneration:
  def __init__(self):
    # publish at 100 Hz
    self.freq = 100
    self.publisher = rospy.Publisher("/follow/target_point", Tracking, queue_size = 10)
    rate = rospy.Rate(self.freq)
    starttime = rospy.get_time()
    print("sending targeting messages")
    while not rospy.is_shutdown():
      msg = generateMessage(rospy.get_time() - starttime)
      self.sendMessage(msg)
      rate.sleep()

  def sendMessage(self, msg):
    self.publisher.publish(msg) 

if __name__=="__main__":
  rospy.init_node("target_generator")
  t = TargetGeneration()
  rospy.spin()
