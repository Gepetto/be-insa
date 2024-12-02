# Copyright 2023 CNRS

# Author: Florent Lamiraux

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test

import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath

from pinocchio.visualize import GepettoVisualizer

class Robot(object):
    # Paths to the model of the robot
    model_dir = "./talos_data"
    mesh_dir = "./talos_data/meshes"
    urdf_model_path = "./talos_data/robots/talos_reduced.urdf"

    def __init__(self):
        self.model, self.collision_model, self.visual_model = \
            pin.buildModelsFromUrdf(self.urdf_model_path, self.mesh_dir, pin.JointModelFreeFlyer())
        self.viz = GepettoVisualizer(self.model, self.collision_model, self.visual_model)
        # Store index of each joint in a dictionary
        self.name_to_config_index = dict()
        self.name_to_joint_index = dict()
        for i in range(0, self.model.njoints):
            joint = self.model.joints[i]
            self.name_to_config_index[self.model.names[i]] = joint.idx_q
            self.name_to_joint_index[self.model.names[i]] = i
        self.leftFootJointId = self.name_to_joint_index["leg_left_6_joint"]
        self.rightFootJointId = self.name_to_joint_index["leg_right_6_joint"]
        self.waistJointId = self.name_to_joint_index["root_joint"]
        # Initialize the viewer.
        try:
            self.viz.initViewer()
        except ImportError as err:
            print("Error while initializing the viewer. It seems you should install gepetto-viewer")
            print(err)
            sys.exit(0)
        try:
            self.viz.loadViewerModel("pinocchio")
        except AttributeError as err:
            print("Error while loading the viewer model. It seems you should start gepetto-viewer")
            print(err)
            sys.exit(0)

    def display(self, q):
        return self.viz.display(q)

if __name__ == "__main__":
    robot = Robot()
    # Display a robot configuration.
    q0 = pin.neutral(robot.model)
    robot.display(q0)
