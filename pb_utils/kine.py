import pybullet as p
import numpy as np

#Kinematics functions

def normalize(x):
    return x/np.linalg.norm(x)

def get_link_base(robot_id, frame_id):
    '''
    Obtain the coordinate of the link frame, according to the convention of pinocchio (at the link origin,
    instead of at the COM as in pybullet)
    '''
    p1 = np.array(p.getLinkState(robot_id,frame_id)[0])
    ori1 = np.array(p.getLinkState(robot_id,frame_id)[1])
    R1 = np.array(p.getMatrixFromQuaternion(ori1)).reshape(3,3)
    p2 = np.array(p.getLinkState(robot_id,frame_id)[2])
    return  p1 - R1.dot(p2), ori1


def get_joint_limits(robot_id, indices):
    lower_limits = []
    upper_limits = []
    for i in indices:
        info = p.getJointInfo(robot_id, i)
        lower_limits += [info[8]]
        upper_limits += [info[9]]
    limits = np.vstack([lower_limits, upper_limits])
    return limits
    
def check_joint_limits(q, joint_limits):
    """
    Return True if within the limit
    """
    upper_check = False in ((q-joint_limits[0]) > 0)
    lower_check = False in ((joint_limits[1]-q) > 0)
    if upper_check or lower_check:
        return False
    else:
        return True
    
def calc_dist_limit(q, joint_limits):
    lower_error = joint_limits[0]-q
    lower_check = (lower_error > 0)
    lower_error = lower_error*lower_check
    upper_error = q-joint_limits[1]
    upper_check = (upper_error > 0)
    upper_error = upper_error*upper_check
    error = lower_error-upper_error
    return error

