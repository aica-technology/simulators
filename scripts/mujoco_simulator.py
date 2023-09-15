"""
This program connects with controllertosim via localhost.
Publishes state information and subscribes control input to and from controllertosim respectively
Launches MuJoCo viewer and visualises robot state
"""
from threading import Thread

import mujoco
from mujoco import viewer

import zmq, clproto
from state_representation import JointState

from network_interfaces.zmq import network
from utilities import receive_encoded_state

context = zmq.Context(1)
publisher = network.configure_publisher(context, '*:6000', True)  
subscriber = network.configure_subscriber(context, '*:6001', True) 

model = mujoco.MjModel.from_xml_path('../universal_robots_ur5e/scene.xml')
data = mujoco.MjData(model)

state_output = JointState().Zero("robot", ['ur5e_' + model.joint(q).name for q in range(model.nq)])

def communication_loop(run):
    # runs continuously, to write command input into mujoco
    # and to pubsub from controllertosim
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

run_thread = True
t = Thread(target=communication_loop, args=[run_thread])
t.start()

viewer.launch(model, data)

run_thread = False
t.join()
