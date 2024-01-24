# Copyright 2018 CNRS

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

import time
import numpy as np
from math import atan2, cos, sin
from pinocchio import centerOfMass, forwardKinematics
from cop_des import CoPDes
from com_trajectory import ComTrajectory
from inverse_kinematics import InverseKinematics
from tools import Constant, Piecewise

# Computes the trajectory of a swing foot.
#
# Input data are
#  - initial and final time of the trajectory,
#  - initial and final pose of the foot,
#  - maximal height of the foot,
#
# The trajectory is polynomial with zero velocities at start and end.
# The orientation of the foot is kept as in intial pose.
class SwingFootTrajectory(object):
    def __init__(self, t_init, t_end, init, end, height):
        assert(init[2] == end[2])
        self.t_init = t_init
        self.t_end = t_end
        self.height = height
        # Write your code here

    def __call__(self, t):
        # write your code here

# Computes a walking whole-body motion
#
# Input data are
#  - an initial configuration of the robot,
#  - a sequence of step positions (x,y,theta) on the ground,
#  - a mapping from time to R corresponding to the desired orientation of the
#    waist. If not provided, keep constant orientation.
#
class WalkingMotion(object):
    step_height = 0.05

    def __init__(self, robot):
        self.robot = robot

    def compute(self, q0, steps, waistOrientation = None):
        # Test input data
        if len(steps) < 4:
            raise RuntimeError("sequence of step should be of length at least 4 instead of " +
                               f"{len(steps)}")
        # Copy steps in order to avoid modifying the input list.
        steps_ = steps[:]
        # Compute offset between waist and center of mass since we control the center of mass
        # indirectly by controlling the waist.
        data = self.robot.model.createData()
        forwardKinematics(self.robot.model, data, q0)
        com = centerOfMass(self.robot.model, data, q0)
        waist_pose = data.oMi[self.robot.waistJointId]
        com_offset = waist_pose.translation - com
        # Trajectory of left and right feet
        self.lf_traj = Piecewise()
        self.rf_traj = Piecewise()
        # write your code here


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from talos import Robot
    from pinocchio import neutral
    import numpy as np
    from inverse_kinematics import InverseKinematics
    import eigenpy

    robot = Robot ()
    ik = InverseKinematics (robot)
    ik.rightFootRefPose.translation = np.array ([0, -0.1, 0.1])
    ik.leftFootRefPose.translation = np.array ([0, 0.1, 0.1])
    ik.waistRefPose.translation = np.array ([0, 0, 0.95])

    q0 = neutral (robot.model)
    q0 [robot.name_to_config_index["leg_right_4_joint"]] = .2
    q0 [robot.name_to_config_index["leg_left_4_joint"]] = .2
    q0 [robot.name_to_config_index["arm_left_2_joint"]] = .2
    q0 [robot.name_to_config_index["arm_right_2_joint"]] = -.2
    q = ik.solve (q0)
    robot.display(q)
    wm = WalkingMotion(robot)
    # First two values correspond to initial position of feet
    # Last two values correspond to final position of feet
    steps = [np.array([0, -.1, 0.]), np.array([0.4, .1, 0.]),
             np.array([.8, -.1, 0.]), np.array([1.2, .1, 0.]),
             np.array([1.6, -.1, 0.]), np.array([1.6, .1, 0.])]
    configs = wm.compute(q, steps)
    for q in configs:
        time.sleep(1e-2)
        robot.display(q)
    delta_t = wm.com_trajectory.delta_t
    times = delta_t*np.arange(wm.com_trajectory.N+1)
    lf = np.array(list(map(wm.lf_traj, times)))
    rf = np.array(list(map(wm.rf_traj, times)))
    cop_des = np.array(list(map(wm.com_trajectory.cop_des, times)))
    fig = plt.figure()
    ax1 = fig.add_subplot(311)
    ax2 = fig.add_subplot(312)
    ax3 = fig.add_subplot(313)
    ax1.plot(times, lf[:,0], label="x left foot")
    ax1.plot(times, rf[:,0], label="x right foot")
    ax1.plot(times, cop_des[:,0], label="x CoPdes")
    ax1.legend()
    ax2.plot(times, lf[:,1], label="y left foot")
    ax2.plot(times, rf[:,1], label="y right foot")
    ax2.plot(times, cop_des[:,1], label="y CoPdes")
    ax2.legend()
    ax3.plot(times, lf[:,2], label="z left foot")
    ax3.plot(times, rf[:,2], label="z right foot")
    ax3.legend()
    plt.show()

