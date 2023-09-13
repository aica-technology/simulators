import time
import _thread

import mujoco
from mujoco import viewer

from state_representation import JointState

model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
data = mujoco.MjData(model)

state_output = JointState().Random("robot", ['ur5e_' + model.joint(q).name for q in range(model.nq)])
control_input = JointState().Random("robot", ['ur5e_' + model.joint(q).name for q in range(model.nq)])

def mujoco_sim():
    viewer.launch(model, data)

def updator():
    i=0
    while True:
        for u in range(model.nu):
            control_input.set_velocity(0, u)
            if u == 0:
                control_input.set_velocity(-2, u)

        for u in range(model.nu):
            data.ctrl[u] = control_input.get_velocity(u)

        for q in range(model.nq):
                    state_output.set_position(data.qpos[q], q)
                    state_output.set_velocity(data.qvel[q], q)
        i+=1

        if i % 10000 == 0:
            print(state_output)
            # print(data.ctrl)
            i=0

try:
   _thread.start_new_thread( mujoco_sim, () )
   _thread.start_new_thread( updator, () )
except:
   print("Error: unable to start thread")

while 1:
   pass
