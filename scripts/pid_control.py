#!/usr/bin/python
import rospy
import numpy as np
import ik_client
import time
import csv
import math

from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
import baxter_tools

joint_names = [ 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

class BaxterCache:
    def __init__(self):
      self.listener = rospy.Subscriber("/robot/joint_states", JointState, self.callback)
      self.joints = None
      self.joints_dot = None

    def callback(self, mesg):
      tmpdict = dict(zip(mesg.name,range(len(mesg.name))))
      self.joints = np.array([mesg.position[tmpdict[joint_name]] for joint_name in joint_names])
      alpha = 0
      if not self.joints_dot is None:
        self.joints_dot = self.joints_dot * alpha + (1-alpha) * np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])
      else:
        self.joints_dot = np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])

class StateLogger:
    def __init__(self, logFileName):
      self.logFile = open(logFileName, 'w')
      self.writer = csv.writer(self.logFile)

    def __enter__(self):
      return self
   
    def __exit__(self, exc_typ, exc_value, traceback):
      print("closing log file")
      self.logFile.close()
 
    def log(self, data):
      self.writer.writerow(data)

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

    # Desired end position (don't care about orientation to start)
    desired_joints = [1.27783743020085, 1.940743404428984, -1.4335451199369675, -0.3248315839206377, 2.0553848590877415, -1.0419260900570144, -2.649420877958835]

    print("Waiting for first joint_state message")
    while bc.joints is None:
      rate.sleep()
    print("First joint_state message acquired")
    cumulativeJointError = np.zeros(bc.joints.shape)
    fileName = "results/logKp%fKd%fKi%fHz%d.csv" % (Kp, Kd, Ki, commandHz)
    print("logging to %s" % fileName)
    with StateLogger(fileName) as logger:
      it = 0
      maxIt = 20*commandHz
      while not rospy.is_shutdown() and it < maxIt:
        # compute the current joint angles
        cur_joints = bc.joints 
        grad = cur_joints - desired_joints 
        # note this needs to be scaled by freq of messages
        cumulativeJointError += grad / commandHz
        grad_dot = bc.joints_dot
        # log at 10 hz
        if it % int(max(1,math.floor(commandHz/10))) == 0:
          logger.log(grad.tolist() +  grad_dot.tolist() + cumulativeJointError.tolist())
          #print(it, commandHz)
        it += 1
        torques = compute_torque(grad, grad_dot, cumulativeJointError, Kp, Kd, Ki)
        messageObj.command = [float(torque) for torque in torques.tolist()]
        pub.publish(messageObj)
        rate.sleep()

def main():
    rospy.init_node('pid_control')
    for commandHz in [10,100,1000]:
      for Ki in [0.1, 10]:
        for Kd in [0.1, 10]:
          for Kp in [100, 10, 1]:
            runExperiment(Kp, Kd, Ki, commandHz)
       

# given joint_grad (the difference desired_joints minus current_joints)
# return the desired joint torque to apply to each joint for a simple
# proportional controller.
def compute_torque(error, error_dot, error_sum, Kp, Kd, Ki):
    torques = (- Kp * error - Kd * error_dot - Ki * error_sum)
    return torques

if __name__ == "__main__":
    main()
