import pybullet as p
import numpy as np
import pinocchio as pin
import transforms3d

#transformation functions
def mat2euler(rot, axes = 'rzyx'):
    return np.array(transforms3d.euler.mat2euler(rot, axes = axes))

def euler2quat(rpy, axes='sxyz'):
    #euler sxyz: used by Manu's codes
    return rectify_quat(transforms3d.euler.euler2quat(*rpy, axes=axes))

def rectify_quat(quat):
    #transform from transforms3d format (w,xyz) to pybullet and pinocchio (xyz, w)
    quat_new = np.concatenate([quat[1:], quat[0:1]])
    return quat_new

def mat2w(rot):
    rot_aa = pin.AngleAxis(rot)
    return rot_aa.angle*rot_aa.axis

def w2quat(q):
    angle = np.linalg.norm(q)
    if abs(angle) < 1e-7:
        ax = np.array([1,0,0])
    else:
        ax, angle = normalize(q), np.linalg.norm(q)
    w = p.getQuaternionFromAxisAngle(ax, angle)
    return np.array(w)

def quat2w(q):
    ax, angle = p.getAxisAngleFromQuaternion(q)
    return np.array(ax)*angle

def w2mat(w):
    angle = np.linalg.norm(w)
    if abs(angle) < 1e-7:
        ax = np.array([1,0,0])
    else:
        ax, angle = w/angle, angle
    R = pin.AngleAxis.toRotationMatrix(pin.AngleAxis(angle, ax))
    return R

