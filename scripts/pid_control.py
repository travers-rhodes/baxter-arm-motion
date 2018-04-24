#!/usr/bin/python
import rospy
import numpy as np
import ik_client
import time

from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState

joint_names = [ 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

class BaxterCache:
    def __init__(self):
      self.listener = rospy.Subscriber("/robot/joint_states", JointState, self.callback)
      self.joints = None
      self.joints_dot = None

    def callback(self, mesg):
      tmpdict = dict(zip(mesg.name,range(len(mesg.name))))
      self.joints = np.array([mesg.position[tmpdict[joint_name]] for joint_name in joint_names])
      alpha = 0.1
      if not self.joints_dot is None:
        self.joints_dot = self.joints_dot * alpha + (1-alpha) * np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])
      else:
        self.joints_dot = np.array([mesg.velocity[tmpdict[joint_name]] for joint_name in joint_names])

def main():
    rospy.init_node('pid_control')
    bc = BaxterCache()
    controlTopic = "/robot/limb/right/joint_command"
    pub = rospy.Publisher(controlTopic, JointCommand)
    messageObj = JointCommand()
    messageObj.mode = JointCommand.TORQUE_MODE
    messageObj.names = joint_names

    # Desired end position (don't care about orientation to start)
    #pos = [0.582583, -0.60819, 0.66003]
    #desired_joints = np.array(ik_client.ik("right",pos))
    desired_joints = [1.27783743020085, 1.940743404428984, -1.4335451199369675, -0.3248315839206377, 2.0553848590877415, -1.0419260900570144, -2.649420877958835]

    # Start out moving the robot arm using the native controller
    messageObj.mode = JointCommand.POSITION_MODE
    messageObj.command = [float(j) for j in desired_joints]
    rate = rospy.Rate(100) #10 Hz
    print("Sending joint commands for 3 seconds...")
    timeout = time.time() + 3 # timeout after 3 seconds 
#    while time.time() < timeout: 
#      pub.publish(messageObj)
#      rate.sleep() 


    print("Waiting for first joint_state message")
    while bc.joints is None:
      rate.sleep()
    print("First joint_state message acquired")
    cumulativeJointError = (desired_joints - bc.joints)
    while not rospy.is_shutdown(): 
      # compute the current joint angles
      cur_joints = bc.joints 
      grad = desired_joints - cur_joints
      cumulativeJointError += grad
      grad_dot = -bc.joints_dot
      print(grad, desired_joints, cumulativeJointError)
      torques = compute_torque(grad, grad_dot, cumulativeJointError)
      messageObj.command = [float(torque) for torque in torques.tolist()]
      pub.publish(messageObj)
      rate.sleep()
       

# given joint_grad (the difference desired_joints minus current_joints)
# return the desired joint torque to apply to each joint for a simple
# proportional controller.
def compute_torque(joint_grad, joint_grad_dot, cumulativeJointError):
    torques = (joint_grad + 0.1 * cumulativeJointError)
    return torques

if __name__ == "__main__":
    main()
