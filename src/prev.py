#!/usr/bin/env python

from __future__ import division

import openravepy

import numpy as np

import time


from lfd.rapprentice import resampling
from lfd.transfer import planning
from lfd.environment.simulation import DynamicSimulationRobotWorld
from lfd.environment.simulation_object import XmlSimulationObject, BoxSimulationObject
from lfd.environment import sim_util


lr = 'r'
manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
ee_link_name = "%s_gripper_tool_frame"%lr
# ee_link = sim.robot.GetLink(ee_link_name)
R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
n_steps = 10


def get_move_traj(t_start, t_end, start_fixed):
    hmat_start = np.r_[np.c_[R, t_start], np.c_[0,0,0,1]]
    hmat_end = np.r_[np.c_[R, t_end], np.c_[0,0,0,1]]
    new_hmats = np.asarray(resampling.interp_hmats(np.arange(n_steps), np.r_[0, n_steps-1], [hmat_start, hmat_end]))
    dof_vals = sim.robot.GetManipulator(manip_name).GetArmDOFValues()
    old_traj = np.tile(dof_vals, (n_steps,1))

    traj, _, _ = planning.plan_follow_traj(sim.robot, manip_name, ee_link, new_hmats, old_traj, start_fixed=start_fixed,
                                           beta_rot=10000.0)
    return traj


shelf_xpos = 0.7
shelf_length = 0.4

table_depth = 0.02
table_b_zpos = 0.8
table_t_zpos = 1.14

# wall_halflength = 0.75
wall_depth = 0.005
# wall_l_ypos = -0.15
wall_l_ypos = -0.15
wall_r_ypos = -0.55

table_ypos = (wall_l_ypos+wall_r_ypos)/2
table_length = abs(wall_r_ypos-wall_l_ypos)

wall_zpos = (table_b_zpos+table_t_zpos)/2
wall_length = abs(table_t_zpos-table_b_zpos)


bucket_zpos = 0.5
bucket_height = 0.3
bucket_wall_depth = 0.02

bucket_f_xpos = 0.3
bucket_b_xpos = 0

bucket_l_ypos = -0.5
bucket_r_ypos = -0.8

bucket_fb_ypos = (bucket_l_ypos+bucket_r_ypos)/2
bucket_fb_length = abs(bucket_r_ypos-bucket_l_ypos)

bucket_lr_xpos = (bucket_f_xpos+bucket_b_xpos)/2
bucket_lr_length = abs(bucket_f_xpos-bucket_b_xpos)


box_edge = 0.05
box0_pos = np.r_[shelf_xpos, table_ypos, table_b_zpos + table_depth/2 + box_edge/2 + 0.005]

# box1_pos = np.r_[.6, 0, table_height+box_depth/2]
sim_objs = []
sim_objs.append(XmlSimulationObject("robots/pr2-beta-static.zae", dynamic=False))
sim_objs.append(XmlSimulationObject("/home/dibyo/workspace/lfd/examples/data/kiva_pod/meshes/pod_lowres.stl"))

# cubby hole
sim_objs.append(BoxSimulationObject("table_b", [shelf_xpos, table_ypos, table_b_zpos],
                                    [shelf_length/2, table_length/2, table_depth/2], dynamic=False))
sim_objs.append(BoxSimulationObject("table_t", [shelf_xpos, table_ypos, table_t_zpos],
                                    [shelf_length/2, table_length/2, table_depth/2], dynamic=False))
sim_objs.append(BoxSimulationObject("wall_l", [shelf_xpos, wall_l_ypos, wall_zpos],
                                    [shelf_length/2, wall_depth/2, wall_length/2], dynamic=False))
sim_objs.append(BoxSimulationObject("wall_r", [shelf_xpos, wall_r_ypos, wall_zpos],
                                    [shelf_length/2, wall_depth/2, wall_length/2], dynamic=False))

# bucket
sim_objs.append(BoxSimulationObject("bucket_f", [bucket_f_xpos, bucket_fb_ypos, bucket_zpos],
                                    [bucket_wall_depth/2, bucket_fb_length/2, bucket_height/2], dynamic=False))
sim_objs.append(BoxSimulationObject("bucket_b", [bucket_b_xpos, bucket_fb_ypos, bucket_zpos],
                                    [bucket_wall_depth/2, bucket_fb_length/2, bucket_height/2], dynamic=False))
sim_objs.append(BoxSimulationObject("bucket_l", [bucket_lr_xpos, bucket_l_ypos, bucket_zpos],
                                    [bucket_lr_length/2, bucket_wall_depth/2, bucket_height/2], dynamic=False))
sim_objs.append(BoxSimulationObject("bucket_r", [bucket_lr_xpos, bucket_r_ypos, bucket_zpos],
                                    [bucket_lr_length/2, bucket_wall_depth/2, bucket_height/2], dynamic=False))
sim_objs.append(BoxSimulationObject("bucket_d", [bucket_lr_xpos, bucket_fb_ypos, bucket_zpos-bucket_height/2],
                                    [bucket_lr_length/2, bucket_fb_length/2, bucket_wall_depth/2], dynamic=False))

sim = DynamicSimulationRobotWorld()
sim.add_objects(sim_objs)

sim.robot.SetDOFValues([0.1], [sim.robot.GetJoint('torso_lift_joint').GetJointIndex()])
sim_util.reset_arms_to_side(sim)

shelf = sim.env.GetKinBody('pod_lowres')
transform = np.array([[0., 0., 1., 0.93],
                      [1., 0., 0., -0.15],
                      [0., 1., 0., 0.],
                      [0., 0., 0., 1.]])
shelf.SetTransform(transform)

ee_link = sim.robot.GetLink(ee_link_name)
sim.env.SetViewer('qtcoin')

move_distance = .7
dof_inds = sim_util.dof_inds_from_name(sim.robot, manip_name)


def move_shelf_back(sim):
    shelf = sim.env.GetKinBody('pod_lowres')
    transform = np.array([[0., 0., 1., 2.],
                          [1., 0., 0., -0.15],
                          [0., 1., 0., 0.],
                          [0., 0., 0., 1.]])
    shelf.SetTransform(transform)


def add_objects_to_simulation(sim):
    objs = list()
    objs.append(BoxSimulationObject("box0", box0_pos, [box_edge/2, box_edge/2, box_edge/2], dynamic=True))
    objs.append(BoxSimulationObject("box1", box0_pos + np.r_[-0.1, -0.1, 0],
                                    [box_edge/2, box_edge/2, box_edge/2], dynamic=True))
    sim.add_objects(objs)
    sim.settle()


def plan_and_execute_path_to_object(sim):
    traj = get_move_traj(box0_pos + np.r_[-move_distance, 0, 0.2], box0_pos + np.r_[0, 0, 0], False)
    sim.execute_trajectory_qtcoin((traj, dof_inds))


def grasp_object(sim):
    sim.close_gripper(lr)


def plan_and_execute_path_to_dropzone(sim):
    traj = get_move_traj(box0_pos + np.r_[0, 0, 0.05], box0_pos + np.r_[-move_distance+0.2, 0, 0.1], True)
    sim.execute_trajectory_qtcoin((traj, dof_inds))

    traj = get_move_traj(box0_pos + np.r_[-move_distance+0.2, 0, 0.1], [bucket_lr_xpos, bucket_fb_ypos,
                                                                        bucket_zpos + bucket_height/2], True)
    sim.execute_trajectory_qtcoin((traj, dof_inds))

    traj = get_move_traj([bucket_lr_xpos, bucket_fb_ypos, bucket_zpos + bucket_height/2],
                         [bucket_lr_xpos, bucket_fb_ypos, bucket_zpos - bucket_height/2 + box_edge/2], True)
    sim.execute_trajectory_qtcoin((traj, dof_inds))


def drop_object(sim):
    sim.open_gripper(lr)
    sim.settle()


def ready_for_next_object(sim):
    sim_util.reset_arms_to_side(sim)


    # # Approach steps
    # move_shelf_back(sim)
    # add_objects_to_simulation(sim)
    # plan_and_execute_path_to_object(sim)
    # grasp_object(sim)
    # plan_and_execute_path_to_dropzone(sim)
    # drop_object(sim)
    # ready_for_next_object(sim)
    # ;)