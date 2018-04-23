#!/usr/bin/python
import rospy
import numpy as np
import ik_client

from baxter_pykdl import baxter_kinematics
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState

joint_names = [ 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

class BaxterCache:
    def __init__(self):
      self.listener = rospy.Subscriber("/robot/joint_states", JointState, self.callback)
      self.joints = None

    def callback(self, mesg):
      self.joints = np.array([dict(zip(mesg.name,mesg.position))[joint_name] for joint_name in joint_names])

def main():
    rospy.init_node('pid_control')
    bc = BaxterCache()
    controlTopic = "/robot/limb/right/joint_command"
    pub = rospy.Publisher(controlTopic, JointCommand)
    messageObj = JointCommand()
    messageObj.mode = JointCommand.TORQUE_MODE
    messageObj.names = joint_names

    kin = baxter_kinematics('right')
    kin.print_kdl_chain()

    # Desired end position (don't care about orientation to start)
    pos = [0.582583, -0.80819, 0.036003]
    ### TODO CHANGE TO SERVICE CALL
    print("This breaks???")
    desired_joints = np.array(ik_client.ik("right",pos))


    rate = rospy.Rate(100) #10 Hz
    print("Waiting for first joint_state message")
    while bc.joints == None:
      rate.sleep()
    print("First joint_state message acquired")
    while not rospy.is_shutdown(): 
      # compute the current joint angles
      cur_joints = bc.joints 
      print(cur_joints, "Cur_joints")
      grad = desired_joints - cur_joints
      print(grad, desired_joints)
      torques = compute_torque(grad)
      messageObj.command = [float(torque) for torque in torques.tolist()]
      pub.publish(messageObj)
      rate.sleep()
       

# given joint_grad (the difference desired_joints minus current_joints)
# return the desired joint torque to apply to each joint for a simple
# proportional controller.
def compute_torque(joint_grad):
    torques = joint_grad * 100000#np.array([100,100000,1000,100000,1,1,1])
    return torques
      
      

if __name__ == "__main__":
    main()
