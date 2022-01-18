import pybullet as p
import numpy as np
import time
import pyscreenshot as ImageGrab

def save_screenshot(x,y,w,h,file_name, to_show='False'):
    # part of the screen
    im=ImageGrab.grab(bbox=(x,y,w,h))
    if to_show:
        im.show()

    # save to file
    im.save(file_name)
    return im

#Kinematics functions

def normalize(x):
    return x/np.linalg.norm(x)

def set_q(robot_id, joint_indices, q, set_base = False):
    if set_base:
        localInertiaPos = np.array(p.getDynamicsInfo(robot_id,-1)[3])
        q_root = q[0:7]
        ori = q_root[3:]
        Rbase = np.array(p.getMatrixFromQuaternion(ori)).reshape(3,3)
        shift_base = Rbase.dot(localInertiaPos)
        pos = q_root[:3]+shift_base
        p.resetBasePositionAndOrientation(robot_id,pos,ori)
        q_joint = q[7:]
    else:
        q_joint = q
    
    #set joint angles
    for i in range(len(q_joint)):
        p.resetJointState(robot_id, joint_indices[i], q_joint[i])

def vis_traj(qs, vis_func, dt=0.1):
    for q in qs:
        vis_func(q)
        time.sleep(dt)

def create_primitives(shapeType=2, rgbaColor=[1, 1, 0, 1], pos = [0, 0, 0], radius = 1, length = 2, halfExtents = [0.5, 0.5, 0.5], baseMass=1, basePosition = [0,0,0]):
    visualShapeId = p.createVisualShape(shapeType=shapeType, rgbaColor=rgbaColor, visualFramePosition=pos, radius=radius, length=length, halfExtents = halfExtents)
    collisionShapeId = p.createCollisionShape(shapeType=shapeType, collisionFramePosition=pos, radius=radius, height=length, halfExtents = halfExtents)
    bodyId = p.createMultiBody(baseMass=baseMass,
                      baseInertialFramePosition=[0, 0, 0],
                      baseVisualShapeIndex=visualShapeId,
                      baseCollisionShapeIndex=collisionShapeId,    
                      basePosition=basePosition,
                      useMaximalCoordinates=True)
    return visualShapeId, collisionShapeId, bodyId


def convexify_obj(name_in, name_out, name_log='log.txt', resolution = 10000000, alpha = 0.04):
    '''
    Code to modify concave objects in pybullet
    Example:
    name_in =  rl.datapath + '/urdf/bookcase_old.obj'
    name_out = rl.datapath + '/urdf/bookcase.obj'
    '''
    p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=resolution )

