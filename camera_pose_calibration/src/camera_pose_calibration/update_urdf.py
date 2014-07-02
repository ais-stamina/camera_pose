#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008-2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import roslib; roslib.load_manifest('calibration_estimation')
import rospy

import math
from urdf_parser_py.urdf import *
import yaml
import numpy
from numpy import matrix, vsplit, sin, cos, reshape, zeros, pi

from calibration_estimation.urdf_params import UrdfParams
from calibration_estimation.joint_chain import JointChain
from calibration_estimation.tilting_laser import TiltingLaser
from calibration_estimation.camera import RectifiedCamera
from calibration_estimation.checkerboard import Checkerboard
from calibration_estimation.single_transform import SingleTransform
from calibration_estimation.single_transform import RPY_to_angle_axis, angle_axis_to_RPY


def diff(v1, v2, eps = 1e-10):
    ''' Determine the difference in two vectors. '''
    if sum( [ math.fabs(x-y) for x,y in zip(v1, v2) ] ) <= eps:
        return 0
    return 1

# URDF updating -- this should probably go in a different file
def update_transmission(urdf, joint, gearing):
    for transmission in urdf.transmissions:
        if transmission.joint == joint:
            transmission.mechanicalReduction = transmission.mechanicalReduction * gearing
            return
    print "No transmission found for:", joint

def update_urdf(urdf, calibrated_params):
    ''' Given urdf and calibrated robot_params, updates the URDF. '''
    joints = list()
    axis = list()
    # update each transmission
    for chain in calibrated_params.chains.values():
        joints += chain._active
        axis += numpy.array(chain._axis)[0,:].tolist()
        for joint, gearing in zip(chain._active, chain._gearing):
            if gearing != 1.0:
                update_transmission(urdf, joint, gearing)
    for laser in calibrated_params.tilting_lasers.values():
        joints.append(laser._config['joint'])
        axis.append(5) # TODO: remove this assumption
        if laser._gearing != 1.0:
            update_transmission(urdf, laser._config['joint'], laser._gearing)

    unchanged_joints = [];

    # update each transform (or joint calibration)
    for joint_name in calibrated_params.transforms.keys():
        link_updated = 0
        try:
            updated_link_params = calibrated_params.transforms[joint_name]._config.T.tolist()[0]
            if diff(updated_link_params[0:3],  urdf.joint_map[joint_name].origin.position):
                print 'Updating xyz for', joint_name, '\n old:', urdf.joint_map[joint_name].origin.position, '\n new:', updated_link_params[0:3]
                urdf.joint_map[joint_name].origin.position = updated_link_params[0:3]
                link_updated = 1
            r1 = RPY_to_angle_axis(urdf.joint_map[joint_name].origin.rotation)
            if diff(r1, updated_link_params[3:6]):
                # TODO: remove assumption that joints are revolute
                if joint_name in joints and urdf.joint_map[joint_name].calibration != None:
                    cal = urdf.joint_map[joint_name].calibration 
                    a = axis[joints.index(joint_name)]
                    a = int(a) - 1
                    print 'Updating calibration for', joint_name, 'by', updated_link_params[a]
                    if cal.rising != None:
                        urdf.joint_map[joint_name].calibration.rising += updated_link_params[a]   
                    if cal.falling != None:
                        urdf.joint_map[joint_name].calibration.falling += updated_link_params[a]
                    link_updated = 1
                else:
                    rot = angle_axis_to_RPY(updated_link_params[3:6])
                    print 'Updating rpy for', joint_name, '\n old:', urdf.joint_map[joint_name].origin.rotation, '\n new:', rot
                    urdf.joint_map[joint_name].origin.rotation = rot
                    link_updated = 1                
        except KeyError:
            print "Joint removed:", joint_name
            print ' xyz:', updated_link_params[0:3]
            print ' rpy:', angle_axis_to_RPY(updated_link_params[3:6])
            link_updated = 1
        if not link_updated:
            unchanged_joints.append( joint_name );
    
    print "The following joints weren't updated: \n", ', '.join(unchanged_joints)
    return urdf
