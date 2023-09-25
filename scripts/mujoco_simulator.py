"""
This program connects with controllertosim via localhost.
Publishes state information and subscribes control input to and from controllertosim respectively
Launches MuJoCo viewer and visualises robot state
"""
import os
from threading import Thread

import mujoco
from mujoco import viewer

import zmq, clproto
from state_representation import JointState

from network_interfaces.zmq import network
from utilities import receive_encoded_state

debug_forcetorque = False

context = zmq.Context(1)
publisher = network.configure_publisher(context, '*:6000', True)  
subscriber = network.configure_subscriber(context, '*:6001', True)

script_dir = os.path.abspath( os.path.dirname( __file__ ) )
model = mujoco.MjModel.from_xml_path(os.path.join(script_dir, os.pardir, "universal_robots_ur5e", "scene.xml"))
data = mujoco.MjData(model)

state_output = JointState().Zero("robot", ['ur5e_' + model.joint(q).name for q in range(model.nq)])

def communication_loop(run):
    # runs continuously, to write command input into mujoco 
    # from a zmq subscriber and send state to a publisher
    i=0
    while run:
        for q in range(model.nq):
            state_output.set_position(data.qpos[q], q)
            state_output.set_velocity(data.qvel[q], q)

        publisher.send(clproto.encode(state_output, clproto.MessageType.JOINT_STATE_MESSAGE))
        message = receive_encoded_state(subscriber)
        if message:
            command = clproto.decode(message)
        
            if command:
                for u in range(model.nu):
                    data.ctrl[u] = command.get_velocity(u)
        # prints forces in x, y, z and torques about x, y, z            
        if debug_forcetorque:
            i+=1
            if i % 30000 == 0: 
                print(int(i/30000), *data.sensor("ft_force").data, *data.sensor("ft_torque").data)


run_thread = True
t = Thread(target=communication_loop, args=[run_thread])
t.start()

mujoco.mj_resetDataKeyframe(model, data, 0)
viewer.launch(model, data)

run_thread = False
t.join()