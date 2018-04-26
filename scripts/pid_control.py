#!/usr/bin/python
import rospy
import numpy as np
import ik_client
import time
import csv
import math

from baxter_core_msgs.msg import JointCommand, EndpointState
from sensor_msgs.msg import JointState
from baxter_arm_motion.msg import Tracking
from visualization_msgs.msg import Marker
import baxter_tools

joint_names = [ 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
Kp = 7
Kd = 0
Ki = 5

class BaxterCache:
    def __init__(self):
      self.trackFuture = False#True#False
      self.listener = rospy.Subscriber("/robot/joint_states", JointState, self.curstate_callback)
      self.desListener = rospy.Subscriber("/follow/target_point", Tracking, self.desired_callback)
      self.truthListener = rospy.Subscriber("/follow/truth", Marker, self.truth_callback)
      self.endListener = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.end_effect_callback)
      self.joints = None
      self.joints_dot = None
      self.desired_pos = None 
      self.desired_pos_next = None
      self.desired_time = None
      self.last_desired_joints = None

    def truth_callback(self, mesg):
      pos = mesg.points[0]
      self.truth = [pos.x,pos.y,pos.z]

    def end_effect_callback(self, mesg):
      pos = mesg.pose.position
      self.end_eff_state = [pos.x,pos.y,pos.z]
    
    def curstate_callback(self, mesg):
      tmpdict = dict(zip(mesg.name,range(len(mesg.name))))
      self.joints = np.array([mesg.position[tmpdict[joint_name]] for joint_name in joint_names])
      self.joints_dot = np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])

    def desired_callback(self, mesg):
      self.desired_pos = np.array([mesg.current.x, mesg.current.y, mesg.current.z])
      self.desired_pos_next = np.array([mesg.future.x, mesg.future.y, mesg.future.z])
      self.desired_time = mesg.curTime

    def get_interp_pos(self):
      curtime = rospy.get_rostime()
      # time stamp is 1 second off, so divide by 1 here ellipsized
      pos = self.desired_pos + (curtime - self.desired_time).to_sec() * (self.desired_pos_next - self.desired_pos)
      print(pos)
      print(self.desired_pos)
      return pos

    def get_desired_joints_and_vel(self):
      if self.desired_pos is None:
        return None
      if self.trackFuture:
        start_joints = self.get_ordered_joints(self.desired_pos)
        if (start_joints is None and self.last_desired_joints is not None):
          return (self.last_desired_joints, self.last_desired_dot)
        elif (start_joints is None):
          return None
        future_joints = self.get_ordered_joints(self.desired_pos_next)
        if (future_joints is None and self.last_desired_joints is not None):
          return (self.last_desired_joints, self.last_desired_dot)
        elif (future_joints is None):
          return None
        interp_pos = self.get_interp_pos()
        #print(interp_pos)
        desired_joints = self.get_ordered_joints(interp_pos)
        # it turns out this linearization is not a good estimate
        # of desired velocity, so instead we go with zero here
        desired_dot = np.zeros(future_joints.shape) #- start_joints
      else: 
        desired_joints = self.get_ordered_joints(self.desired_pos)
        if (desired_joints is None and self.last_desired_joints is not None):
          return (self.last_desired_joints, self.last_desired_dot)
        elif (desired_joints is None):
          return None
        desired_dot = np.zeros(desired_joints.shape) 
      self.last_desired_joints = desired_joints
      self.last_desired_dot = desired_dot
      return (desired_joints, desired_dot)

    def get_ordered_joints(self, pos):
      ik_resp = ik_client.ik("right",pos)
      if ik_resp == 0:
        return
      ik_resp_joints = ik_resp[0]
      ik_resp_names = ik_resp[1]
      tmpdict = dict(zip(ik_resp_names,range(len(ik_resp_names))))
      return np.array([ik_resp_joints[tmpdict[nm]] for nm in joint_names])

class StateLogger:
    def __init__(self, logFileName):
      self.mock = False
      if not self.mock:
        self.logFile = open(logFileName, 'w')
        self.writer = csv.writer(self.logFile)

    def __enter__(self):
      return self
   
    def __exit__(self, exc_typ, exc_value, traceback):
      print("closing log file")
      if not self.mock:
        self.logFile.close()
 
    def log(self, data):
      if not self.mock:
        self.writer.writerow(data)
      else:
        print(data)

def runExperiment(Kp, Kd, Ki, commandHz):
    bc = BaxterCache()

    print("Resetting robot")
    tucker = baxter_tools.Tuck(False)
    rospy.on_shutdown(tucker.clean_shutdown)
    tucker.supervised_tuck()
    rospy.sleep(1)

    controlTopic = "/robot/limb/right/joint_command"
    pub = rospy.Publisher(controlTopic, JointCommand)
    messageObj = JointCommand()
    messageObj.mode = JointCommand.TORQUE_MODE
    messageObj.names = joint_names

    rate = rospy.Rate(commandHz)
    

    print("Waiting for first joint_state message")
    while bc.joints is None:
      rate.sleep()
    print("First joint_state message acquired")
    
    print("Waiting for first target_loc message")
    while bc.get_desired_joints_and_vel() is None:
      rate.sleep()
    print("First target_loc message acquired")
    time.sleep(1)
    
    cumulativeJointError = np.zeros(bc.joints.shape)
    fileName = "results/Results%s.csv" % ("Interp" if bc.trackFuture else "Naive")
    print("logging to %s" % fileName)
    with StateLogger(fileName) as logger:
      it = 0
      maxIt = 2000*commandHz
      while not rospy.is_shutdown():# and it < maxIt:
        des = bc.get_desired_joints_and_vel()
        if des is None or des[0] is None:
          continue
        desired_joints = des[0]
        desired_dot = des[1]
        # compute the current joint angles
        err = bc.joints - desired_joints 
        # note this needs to be scaled by freq of messages
        cumulativeJointError += err / commandHz
        err_dot = bc.joints_dot - desired_dot
        # log at 40 hz
        if it % int(max(1,math.floor(commandHz/40))) == 0:
          logger.log([rospy.get_rostime()] + bc.end_eff_state + bc.truth)
        it += 1
        torques = compute_torque(err, err_dot, cumulativeJointError, Kp, Kd, Ki)
        messageObj.command = [float(torque) for torque in torques.tolist()]
        pub.publish(messageObj)
        rate.sleep()

def main():
    rospy.init_node('pid_control')
    commandHz = 1000
    runExperiment(Kp, Kd, Ki, commandHz)
       

# given joint_grad (the difference desired_joints minus current_joints)
# return the desired joint torque to apply to each joint for a simple
# proportional controller.
def compute_torque(error, error_dot, error_sum, Kp, Kd, Ki):
    # clip torques to avoid large torques
    maxtorque = 30
    torques = (- Kp * error - Kd * error_dot - Ki * error_sum)
    torques = np.array([min(max(t,-maxtorque),maxtorque) for t in torques])
    return torques

if __name__ == "__main__":
    main()
