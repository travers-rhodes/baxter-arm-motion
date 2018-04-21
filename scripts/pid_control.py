#!/usr/bin/python
import rospy
import numpy as np

from baxter_pykdl import baxter_kinematics
from baxter_core_msgs.msg import JointCommand


def main():
    rospy.init_node('pid_control')
    controlTopic = "/robot/limb/right/joint_command"
    pub = rospy.Publisher(controlTopic, JointCommand)
    messageObj = JointCommand()
    messageObj.mode = JointCommand.TORQUE_MODE
    messageObj.names = [ 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

    kin = baxter_kinematics('right')
    kin.print_kdl_chain()

    # Desired end position (don't care about orientation to start)
    pos = [0.582583, -0.180819, 0.216003]
    desired_joints = np.array(kin.inverse_kinematics(pos))

    rate = rospy.Rate(10) #10 Hz
    while not rospy.is_shutdown(): 
      # compute the current joint angles
      cur_joints = np.array(kin.forward_position_kinematics())
      grad = desired_joints - cur_joints
      torques = compute_torque(grad)
      messageObj.command = [float(torque) for torque in torques.tolist()]
      pub.publish(messageObj)
      rate.sleep()
       

# given joint_grad (the difference desired_joints minus current_joints)
# return the desired joint torque to apply to each joint for a simple
# proportional controller.
def compute_torque(joint_grad):
    torques = joint_grad * 10.0
    return torques
      
      

if __name__ == "__main__":
    main()
