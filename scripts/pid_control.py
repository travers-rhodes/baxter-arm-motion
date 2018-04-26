#!/usr/bin/python
import rospy
import numpy as np
import ik_client
import time
import csv
import math

from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from baxter_arm_motion.msg import Tracking
import baxter_tools

joint_names = [ 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

class BaxterCache:
    def __init__(self):
      self.listener = rospy.Subscriber("/robot/joint_states", JointState, self.curstate_callback)
      self.desListener = rospy.Subscriber("/follow/target_point", Tracking, self.desired_callback)
      self.joints = None
      self.joints_dot = None
      self.desired_joints = [1.27783743020085, 1.940743404428984, -1.4335451199369675, -0.3248315839206377, 2.0553848590877415, -1.0419260900570144, -2.649420877958835]

    def curstate_callback(self, mesg):
      tmpdict = dict(zip(mesg.name,range(len(mesg.name))))
      self.joints = np.array([mesg.position[tmpdict[joint_name]] for joint_name in joint_names])
      alpha = 0
      if not self.joints_dot is None:
        self.joints_dot = self.joints_dot * alpha + (1-alpha) * np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])
      else:
        self.joints_dot = np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])

    def desired_callback(self, mesg):
      pos = [mesg.current.x, mesg.current.y, mesg.current.z]
      print(pos)
      ik_resp = ik_client.ik("right",pos)
      ik_resp_joints = ik_resp[0]
      ik_resp_names = ik_resp[1]
      tmpdict = dict(zip(ik_resp_names,range(len(ik_resp_names))))
      self.desired_joints = [ik_resp_joints[tmpdict[nm]] for nm in joint_names]
       

class StateLogger:
    def __init__(self, logFileName):
      self.mock = True
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
    
    cumulativeJointError = np.zeros(bc.joints.shape)
    fileName = "results/logKp%fKd%fKi%fHz%d.csv" % (Kp, Kd, Ki, commandHz)
    print("logging to %s" % fileName)
    with StateLogger(fileName) as logger:
      it = 0
      maxIt = 2000*commandHz
      while not rospy.is_shutdown() and it < maxIt:
        # compute the current joint angles
        err = bc.joints - bc.desired_joints 
        # note this needs to be scaled by freq of messages
        cumulativeJointError += err / commandHz
        err_dot = bc.joints_dot
        # log at 10 hz
        if it % int(max(1,math.floor(commandHz/10))) == 0:
          logger.log(err.tolist() +  err_dot.tolist() + cumulativeJointError.tolist())
          #print(it, commandHz)
        it += 1
        torques = compute_torque(err, err_dot, cumulativeJointError, Kp, Kd, Ki)
        messageObj.command = [float(torque) for torque in torques.tolist()]
        pub.publish(messageObj)
        rate.sleep()

def main():
    rospy.init_node('pid_control')
    for commandHz in [100]:
      for Ki in [0.1]:
        for Kd in [1]:
          for Kp in [10]:
            runExperiment(Kp, Kd, Ki, commandHz)
       

# given joint_grad (the difference desired_joints minus current_joints)
# return the desired joint torque to apply to each joint for a simple
# proportional controller.
def compute_torque(error, error_dot, error_sum, Kp, Kd, Ki):
    torques = (- Kp * error - Kd * error_dot - Ki * error_sum)
    return torques

if __name__ == "__main__":
    main()
