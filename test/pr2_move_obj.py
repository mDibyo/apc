from __future__ import division

import numpy as np
import os.path as osp

from lfd.environment.simulation import DynamicSimulationRobotWorld
from lfd.environment.simulation_object import XmlSimulationObject, BoxSimulationObject
from lfd.environment import sim_util
from lfd.environment import environment

from test_utils import *
from utils import DATA_DIRECTORY

sim_objs = []
sim_objs.append(XmlSimulationObject("robots/pr2-beta-static.zae", dynamic=False))
sim_objs.append(XmlSimulationObject(osp.join(DATA_DIRECTORY, "models", "cubbyhole_all_combined.kinbody.xml"), dynamic=False))
sim_objs.append(BoxSimulationObject("obj", [-0.15944789, -0.31466704,  0.917779], [0.080517,0.036479,0.097779], dynamic=True))


sim = DynamicSimulationRobotWorld()
sim.add_objects(sim_objs)
sim.create_viewer()

r = sim.robot
m = sim.robot.GetManipulator("rightarm_torso")
resetRobot(r)
r.SetDOFValues(, m.GetArmIndices())
r.SetDOFValues([0], [34])

start_joints = np.array([ 0., -0.14371035, 0.9, 0.1, -1.2,-2.35495463, -0.24157068,  2.27367817])
end_joints = [ 0., -0.14371035, 0.32910031, 0.1, -0.48660307,-2.35495463, -0.24157068,  2.27367817]
traj = np.vstack([np.linspace(s,e,5) for s,e in zip(start_joints,end_joints)]).T

env = environment.LfdEnvironment(sim, sim)

shelf = sim.sim_objs[1].get_bullet_objects()[0]
shelf_pose = np.eye(4)
shelf_pose[2,3] = 0.796300 
shelf.GetKinBody().SetTransform(shelf_pose)
shelf.UpdateBullet()

sim.update()
sim.viewer.Idle()
raw_input("start?")
sim.settle(max_steps=1000)
sim.viewer.Idle()
