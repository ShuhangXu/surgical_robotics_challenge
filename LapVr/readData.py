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

#port = Amp.FirewirePort(0)
port = Amp.PortFactory("")
board = Amp.AmpIO(6)
port.AddBoard(board)

while not rospy.is_shutdown():
   print(GetEncoders())
   time.sleep(0.1)
   

