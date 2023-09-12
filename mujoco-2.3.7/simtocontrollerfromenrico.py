import time

import mujoco
from mujoco import viewer
from threading import Thread

import zmq
from state_representation import CartesianState, JointState

from network_interfaces.zmq import network

context = zmq.Context(1)
publisher = network.configure_publisher(context, '172.0.0.1:6000', False)  # state
subscriber = network.configure_subscriber(context, '172.0.0.1:6001', False)  # command

model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
data = mujoco.MjData(model)

state = network.StateMessage()
state.joint_state = JointState().Random("robot", ['ur5e_' + model.joint(q).name for q in range(model.nq)])
print(state.joint_state)

def communication_loop(run):
    while run:
        for q in range(model.nq):
            state.joint_state.set_position(data.qpos[q], q)
            state.joint_state.set_velocity(data.qvel[q], q)
        print(state.joint_state)
        network.send_state(state, publisher)
        command = network.receive_command(subscriber)
        if command:
            for u in range(model.nu):
                data.ctrl[u] = command.joint_state.get_velocity(u)
        time.sleep(1.0)

run_thread = True
t = Thread(target=communication_loop, args=[run_thread])
t.start()

viewer.launch(model, data)

run_thread = False
t.join()