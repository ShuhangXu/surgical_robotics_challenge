#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
from surgical_robotics_challenge.kinematics.psmIK import *
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.utils.obj_control_gui import ObjectGUI
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
import Amp1394Python as Amp
from psm_arm import PSM
from simulation_manager import SimulationManager
import rospy
import time

def GetEncoders():
   port.ReadAllBoards()
   encoder = [0,0,0,0]
   for i in range(0,4):
       encoder[i] = board.GetEncoderPosition(i)
   return encoder

def Cleanup():
   port.RemoveBoard(board.GetBoardId())

def map_value(x, min_input, max_input, min_output, max_output):
    return (x - min_input) / (max_input - min_input) * (max_output - min_output) + min_output


class PSMController:
    def __init__(self, arm):
        self.arm = arm

    def update_arm_pose(self):
        data = GetEncoders()
        yaw = data[0]
        x = data[3] # Left, right
        y = data[1] # Up,down
        z = data[2] # In, out
        yaw = map_value(yaw, 878, -5304, 3.14, 0)
        x = map_value(x, 9780, 0, -0.15, 0.15)
        y = map_value(y, -9583, 0, -0.15, 0.15)
        z = map_value(z, -7476, 14272, -0.05, -0.20)
        T_t_b = Frame(Rotation.RPY(3.14, 0, yaw), Vector(x, y, z))
        self.arm.servo_cp(T_t_b)
        # self.arm.set_jaw_angle(gui.gr)
        # self.arm.run_grasp_logic(gui.gr)
        print(str(x) + " " + str(y) + " " + str(z) + " " + str(yaw) + "\n")

    def run(self):
        self.update_arm_pose()


class ECMController:
    def __init__(self, gui, ecm):
        self.counter = 0
        self._ecm = ecm
        self._cam_gui = gui

    def update_camera_pose(self):
        self._cam_gui.App.update()
        self._ecm.servo_jp(self._cam_gui.jnt_cmds)

    def run(self):
            self.update_camera_pose()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False)
    parser.add_argument('--ecm', action='store', dest='run_ecm', help='Control ECM', default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    parsed_args.run_ecm = get_boolean_from_opt(parsed_args.run_ecm)

    simulation_manager = SimulationManager(parsed_args.client_name)

    time.sleep(0.5)
    controllers = []
    port = Amp.PortFactory("")
    board = Amp.AmpIO(6)
    port.AddBoard(board)

    if parsed_args.run_psm_one is True:
        arm_name = 'psm1'
        psm = PSM(simulation_manager, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM1
            # init_xyz = [0.1, -0.85, -0.15]
            controller = PSMController(psm)
            controllers.append(controller)

    if len(controllers) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            time.sleep(0.005)
