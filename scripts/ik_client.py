#!/usr/bin/env python

"""
Copied from Baxter RSDK Inverse Kinematics Example
"""
import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
'''
For video:
    x: 0.672040974663
    y: -0.724013073822
    z: 0.0569658745728
  orientation: 
    x: 0.0445988093722
    y: 0.690662336673
    z: 0.0326806693118
    w: 0.721060647073

'''
'''
 position: 
    x: 0.406484166471
    y: -0.363021194299
    z: 0.11514337617
  orientation: 
    x: -0.0674629532155
    y: 0.996194308015
    z: 0.045673634235
    w: 0.030976922984
'''


# limb should be "left" or "right"
# position should be [xpos, ypos, zpos]
def ik(limb, position):
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                ),
                orientation=Quaternion(
                   x= 0.0445988093722,
                   y= 0.690662336673,
                   z= 0.0326806693118,
                   w= 0.721060647073
                ),
            ),
        )

    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        limb_joints = resp.joints[0].position
        return (limb_joints, resp.joints[0].name)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    return ik_test(args.limb)

if __name__ == '__main__':
    sys.exit(main())
